"""Stage and supervise unchanged official Linux physics/controller processes.

No run_sim.sh, shell command interpolation, pkill, source config edits, or
process-name termination. Only Popen children owned by this instance are
terminated. The official eCAL topics are global: any existing official
simulator/controller blocks startup, even if it belongs to another project.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import stat
import struct
import subprocess
import sys
import time


class OfficialRuntimeError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _yaml_scalars(path, values):
    text = path.read_text(encoding="utf-8")
    for key, value in values.items():
        replacement = json.dumps(value) if isinstance(value, str) else str(value)
        expression = re.compile(r"(?m)^(\s*" + re.escape(key) + r"\s*:\s*)[^#\r\n]+")
        text, count = expression.subn(lambda match: match.group(1) + replacement + " ", text)
        if count != 1:
            raise OfficialRuntimeError(f"Expected exactly one {key!r} in copied config {path}")
    path.write_text(text, encoding="utf-8", newline="\n")


class OfficialRuntime:
    """Explicit lifecycle for a per-run copy of the official runtime.

    preflight() returns a report with ready/errors. prepare() copies/hashes but
    does not execute binaries, and can be inspected on Windows. start() always
    repeats the Linux/dependency/conflict checks and refuses an unsafe start.
    workdir must live below this repository's task3_sim, never below src/deps.
    A supplied scene is copied byte for byte; its resource paths must already
    be valid on the target host. Its source directory and original sources are
    never changed by this module.
    """

    def __init__(self, repo_root, workdir, scene_path, sdk_port=44888, *,
                 run_command=subprocess.run, popen=subprocess.Popen,
                 clock=time.monotonic, wait=time.sleep):
        self.repo_root = Path(repo_root).resolve()
        self.workdir = Path(workdir).resolve()
        self.scene_path = Path(scene_path).resolve()
        if not self.workdir.is_relative_to(self.repo_root / "task3_sim") or self.workdir == self.repo_root / "task3_sim":
            raise ValueError("Official runtime workdir must be a new child directory of repo_root/task3_sim")
        if not isinstance(sdk_port, int) or isinstance(sdk_port, bool) or not 1024 <= sdk_port <= 65535:
            raise ValueError("sdk_port must be an integer from 1024 through 65535")
        self.sdk_port = sdk_port
        self.run_command, self.popen, self.clock, self.wait = run_command, popen, clock, wait
        self.sim_source = self.repo_root / "src/robot_mujoco"
        self.mc_source = self.repo_root / "src/robot_mc/build/export"
        self.sim_binary = self.workdir / "simulate/build/robot_mujoco"
        self.mc_binary = self.workdir / "mc/build/export/mc/bin/mc_ctrl"
        self.staged_scene = self.workdir / "zsibot_robots/xgb/task3.xml"
        self.processes = {}
        self.logs = {}
        self.manifest = None
        self.preflight_report = None
        self._log_handles = {}
        self._global_lock = None
        self._closed = False
        self._process_startup_completed = False
        self._startup_error = None

    def _environment(self):
        env = os.environ.copy()
        library_dir = self.mc_binary.parent if self.mc_binary.exists() else self.mc_source / "mc/bin"
        previous = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = str(library_dir) + (os.pathsep + previous if previous else "")
        env["ROBOT_TYPE"] = "XG"
        # Official librobot reads this independently of sdk_config.yaml.
        # Its absent-variable default is the hardware WiFi IP 192.168.234.1,
        # which cannot bind on a normal simulation host. This runtime owns a
        # same-host SDK session, so scope the official override to its children.
        env["SDK_CLIENT_IP"] = "127.0.0.1"
        return env

    def _existing_official_processes(self):
        completed = self.run_command(["ps", "-eo", "pid=,comm="], capture_output=True, text=True,
                                     timeout=10, check=False)
        if completed.returncode:
            raise OfficialRuntimeError("Cannot inspect running processes; eCAL isolation cannot be verified")
        own_pids = {process.pid for process in self.processes.values()}
        found = []
        for line in completed.stdout.splitlines():
            parts = line.split(None, 1)
            if len(parts) == 2 and parts[0].isdigit() and parts[1].strip() in ("mc_ctrl", "robot_mujoco"):
                pid = int(parts[0])
                if pid not in own_pids:
                    found.append({"pid": pid, "name": parts[1].strip()})
        return found

    def _native_supported(self):
        return (sys.platform == "linux" and platform.machine().lower() in ("x86_64", "amd64")
                and platform.python_implementation() == "CPython" and sys.version_info[:2] == (3, 10))

    def preflight(self, bridge_executable=None):
        errors, warnings, libraries = [], [], {}
        native_supported = self._native_supported()
        if not native_supported:
            errors.append("Official x86_64 runtime and supplied SDK require Ubuntu/Linux x86_64 CPython 3.10")
        required = {
            "robot_mujoco": self.sim_source / "simulate/build/robot_mujoco",
            "sim_config": self.sim_source / "simulate/config.yaml",
            "robot_model": self.sim_source / "zsibot_robots/xgb/xgb.xml",
            "mc_ctrl": self.mc_source / "mc/bin/mc_ctrl",
            "mc_parameters": self.mc_source / "config/xg-user-parameters.yaml",
            "sdk_config": self.mc_source / "config/sdk_config.yaml",
            "sdk_library": self.repo_root / "deps/zsibot_sdk/lib/zsl-1/x86_64/libmc_sdk_zsl_1_x86_64.so",
            "sdk_binding": self.repo_root / "deps/zsibot_sdk/lib/zsl-1/x86_64/mc_sdk_zsl_1_py.cpython-310-x86_64-linux-gnu.so",
            "task3_scene": self.scene_path,
        }
        if bridge_executable is not None:
            required["state_bridge"] = Path(bridge_executable).resolve()
        else:
            warnings.append("State bridge not checked: pass its compiled executable before startup")
        for name, path in required.items():
            if not path.is_file():
                errors.append(f"Missing {name}: {path}")
        executables = {name: path for name, path in required.items()
                       if name in ("robot_mujoco", "mc_ctrl", "state_bridge", "sdk_library", "sdk_binding")}
        for name, path in executables.items():
            if not path.is_file():
                continue
            try:
                with path.open("rb") as source:
                    header = source.read(20)
            except OSError as exc:
                errors.append(f"Cannot read {name}: {exc}")
                continue
            if len(header) < 20 or header[:6] != b"\x7fELF\x02\x01" or struct.unpack("<H", header[18:20])[0] != 62:
                errors.append(f"{name} is not the expected Linux x86_64 ELF executable: {path}")
                continue
            if native_supported:
                if name == "state_bridge" and not os.access(path, os.X_OK):
                    errors.append(f"State bridge is not executable: {path}")
                elif name in ("robot_mujoco", "mc_ctrl") and not os.access(path, os.X_OK):
                    warnings.append(f"{name} source lacks execute permission; owner execute is set on the staged copy only")
                try:
                    result = self.run_command(["ldd", str(path)], env=self._environment(), capture_output=True,
                                              text=True, timeout=15, check=False)
                    libraries[name] = result.stdout + result.stderr
                    if result.returncode != 0 or "not found" in libraries[name]:
                        errors.append(f"{name} has unresolved native dependencies; see report.libraries.{name}")
                except (OSError, subprocess.TimeoutExpired) as exc:
                    errors.append(f"Cannot check {name} native dependencies with ldd: {exc}")
        conflicts = []
        if native_supported:
            if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
                errors.append("Official robot_mujoco requires an available desktop DISPLAY/WAYLAND_DISPLAY")
            try:
                conflicts = self._existing_official_processes()
                if conflicts:
                    errors.append("Other official mc_ctrl/robot_mujoco processes are active; fixed eCAL topics cannot be isolated")
            except (OSError, subprocess.TimeoutExpired, OfficialRuntimeError) as exc:
                errors.append(str(exc))
            # A legacy SDK client can also target the controller's fixed UDP
            # server, even when task3 uses a new client feedback port.
            for port in sorted({self.sdk_port, 43988, 43997, 43900}):
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                        probe.bind(("127.0.0.1", port))
                except OSError as exc:
                    errors.append(f"Official SDK/controller loopback UDP port {port} is unavailable: {exc}")
        warnings.append("This preflight does not prove gait stability or successful source-state delivery")
        self.preflight_report = {"ready": not errors, "errors": errors, "warnings": warnings,
                                 "platform": platform.platform(), "python": platform.python_version(),
                                 "required_files": {key: str(value) for key, value in required.items()},
                                 "conflicting_processes": conflicts, "libraries": libraries,
                                 "sdk_client_ip": "127.0.0.1", "sdk_client_port": self.sdk_port,
                                 "sdk_controller_bind_ip": self._environment()["SDK_CLIENT_IP"],
                                 "sdk_controller_bind_environment": "SDK_CLIENT_IP",
                                 "ecal_topics": ["mujoco_state", "mujoco_cmd"], "official_sources_modified": False}
        return self.preflight_report

    def _copy_file(self, source, destination, rows):
        destination.parent.mkdir(parents=True, exist_ok=True)
        before = sha256(source)
        shutil.copy2(source, destination, follow_symlinks=True)
        after = sha256(destination)
        if before != after or sha256(source) != before:
            raise OfficialRuntimeError(f"Copy verification failed or source changed during copy: {source}")
        rows.append({"source": str(source), "destination": str(destination), "source_sha256": before,
                     "staged_initial_sha256": after})

    def _copy_tree(self, source, destination, rows):
        if not source.is_dir():
            raise OfficialRuntimeError(f"Required official runtime resource directory is missing: {source}")
        for entry in sorted(source.rglob("*")):
            if ".git" in entry.relative_to(source).parts:
                continue
            if entry.is_symlink() and entry.is_dir():
                raise OfficialRuntimeError(f"Directory symlink needs explicit resource review before staging: {entry}")
            if entry.is_file():
                self._copy_file(entry, destination / entry.relative_to(source), rows)

    def _verify_sources(self):
        if self.manifest is None:
            return True
        changed = [row["source"] for row in self.manifest["copies"]
                   if not Path(row["source"]).is_file() or sha256(row["source"]) != row["source_sha256"]]
        self.manifest["sources_unchanged"] = not changed
        self.manifest["changed_sources"] = changed
        return not changed

    def prepare(self):
        if self.manifest is not None:
            if not self._verify_sources():
                raise OfficialRuntimeError("Official source files changed after staging")
            return self.manifest
        if self.workdir.exists() and any(self.workdir.iterdir()):
            raise OfficialRuntimeError("Refusing to overwrite a nonempty official runtime workdir")
        self.workdir.mkdir(parents=True, exist_ok=True)
        rows = []
        self._copy_file(self.sim_source / "simulate/build/robot_mujoco", self.sim_binary, rows)
        plugin_source = self.sim_source / "simulate/build/mujoco_plugin"
        if plugin_source.is_dir():
            self._copy_tree(plugin_source, self.sim_binary.parent / "mujoco_plugin", rows)
        self._copy_file(self.sim_source / "simulate/config.yaml", self.workdir / "simulate/config.yaml", rows)
        self._copy_tree(self.sim_source / "zsibot_robots", self.workdir / "zsibot_robots", rows)
        self._copy_tree(self.mc_source, self.workdir / "mc/build/export", rows)
        for binary in (self.sim_binary, self.mc_binary):
            binary.chmod(binary.stat().st_mode | stat.S_IXUSR)
        if self.staged_scene.exists():
            raise OfficialRuntimeError("Official source already contains task3.xml; refusing an ambiguous scene overwrite")
        self._copy_file(self.scene_path, self.staged_scene, rows)
        for config in (self.workdir / "simulate/config.yaml", self.workdir / "mc/build/export/config/xg-user-parameters.yaml",
                       self.workdir / "mc/build/export/config/sdk_config.yaml"):
            config.chmod(config.stat().st_mode | stat.S_IWUSR)
        _yaml_scalars(self.workdir / "simulate/config.yaml", {"robot": "xgb", "robot_scene": "task3.xml",
                      "robot_path": "zsibot_robots", "print_scene_information": 0, "enable_elastic_band": 0})
        _yaml_scalars(self.workdir / "mc/build/export/config/xg-user-parameters.yaml", {"motor_platform_type": 5})
        _yaml_scalars(self.workdir / "mc/build/export/config/sdk_config.yaml", {"target_ip": "127.0.0.1", "target_port": self.sdk_port})
        for row in rows:
            row["staged_final_sha256"] = sha256(row["destination"])
            row["derived_config_changed"] = row["staged_final_sha256"] != row["staged_initial_sha256"]
        self.manifest = {"copies": rows, "sdk_port": self.sdk_port, "model_scene": str(self.staged_scene),
                         "official_sources_modified": False,
                         "scope": "Unchanged official executables/resources copied; only staged configs select task3 and motor_platform_type=5"}
        if not self._verify_sources():
            raise OfficialRuntimeError("Source changed during runtime staging; no processes started")
        (self.workdir / "staging_manifest.json").write_text(json.dumps(self.manifest, indent=2) + "\n", encoding="utf-8")
        return self.manifest

    def _acquire_global_lock(self):
        import fcntl
        lock_path = Path("/tmp") / f"robotac_task3_official_{os.getuid()}.lock"
        self._global_lock = lock_path.open("a")
        try:
            fcntl.flock(self._global_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._global_lock.close()
            self._global_lock = None
            raise OfficialRuntimeError("Another task3 official runtime owns the global eCAL startup lock") from exc

    def _launch(self, name, arguments, cwd):
        log_path = self.workdir / f"{name}.log"
        handle = log_path.open("wb")
        self._log_handles[name], self.logs[name] = handle, log_path
        self.processes[name] = self.popen([str(x) for x in arguments], cwd=str(cwd), env=self._environment(),
                                          stdin=subprocess.DEVNULL, stdout=handle, stderr=subprocess.STDOUT,
                                          start_new_session=True)

    def _wait_bridge(self, timeout_s=10.0):
        deadline = float(self.clock()) + timeout_s
        while True:
            process = self.processes["official_state_bridge"]
            if process.poll() is not None:
                raise OfficialRuntimeError(f"State bridge exited before readiness; see {self.logs['official_state_bridge']}")
            if "TASK3_OFFICIAL_STATE_READY" in self.logs["official_state_bridge"].read_text(encoding="utf-8", errors="replace"):
                return
            if float(self.clock()) >= deadline:
                raise OfficialRuntimeError("Timed out waiting for official state bridge subscriber readiness")
            self.wait(0.025)

    def start(self, state_port, bridge_executable):
        if self._closed or self.processes:
            raise OfficialRuntimeError("Runtime has already been started or closed")
        if not isinstance(state_port, int) or isinstance(state_port, bool) or not 1024 <= state_port <= 65535 or state_port == self.sdk_port:
            raise ValueError("state_port must be a distinct loopback UDP port from 1024 through 65535")
        bridge = Path(bridge_executable).resolve()
        try:
            self._acquire_global_lock()
            report = self.preflight(bridge)
            if not report["ready"]:
                raise OfficialRuntimeError("Official runtime preflight failed: " + "; ".join(report["errors"]))
            self.prepare()
            (self.workdir / "preflight.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            self._launch("official_state_bridge", [bridge, "--port", str(state_port)], self.workdir)
            self._wait_bridge()
            # The binary also reads ../config.yaml before its argv scene.
            self._launch("robot_mujoco", [self.sim_binary, self.staged_scene], self.sim_binary.parent)
            self.wait(0.25)
            self.check_alive()
            self._launch("mc_ctrl", [self.mc_binary, "r"], self.mc_binary.parent)
            self.wait(0.25)
            self.check_alive()
            self._process_startup_completed = True
            return {name: process.pid for name, process in self.processes.items()}
        except Exception as exc:
            self._startup_error = str(exc)
            try:
                self.close()
            except OfficialRuntimeError:
                # close writes its errors to runtime_status.json. Preserve the
                # original startup failure as the raised explanation.
                pass
            raise

    def check_alive(self):
        if not self.processes:
            raise OfficialRuntimeError("Official runtime has no owned processes")
        for name, process in self.processes.items():
            code = process.poll()
            if code is not None:
                raise OfficialRuntimeError(f"Owned {name} exited with code {code}; see {self.logs[name]}")
        controller_log = self.logs.get("mc_ctrl")
        if controller_log is not None:
            # mc_ctrl can remain alive and initialize its gait FSM after its
            # SDK sockets fail. A live PID therefore does not prove control
            # readiness. The startup error is at the beginning of its log.
            with controller_log.open("rb") as source:
                startup = source.read(65536).lower()
            if b"sdk init failed" in startup or b"bind: cannot assign requested address" in startup:
                reported = re.search(rb"client_ip:\s*([^\s]+)", startup)
                actual_ip = reported.group(1).decode("ascii", errors="replace") if reported else "unknown"
                raise OfficialRuntimeError(
                    f"Official mc_ctrl SDK initialization failed (reported client_ip={actual_ip}); expected child "
                    "SDK_CLIENT_IP=127.0.0.1 and free loopback UDP ports 43997/43900. "
                    f"See {controller_log}")
        return True

    def report(self):
        """JSON-safe current evidence; this never upgrades preflight to a run."""
        return {
            "workdir": str(self.workdir), "prepared": self.manifest is not None,
            "closed": self._closed, "preflight": self.preflight_report,
            "all_official_processes_launched": self._process_startup_completed,
            "startup_error": self._startup_error,
            "readiness_scope": "Owned process startup only; actual SDK connection and physics feedback are checked separately",
            "processes": {name: {"pid": process.pid, "exit_code": process.poll(), "log": str(self.logs[name])}
                          for name, process in self.processes.items()},
            "staging_manifest": str(self.workdir / "staging_manifest.json") if self.manifest is not None else None,
            "source_files_unchanged_at_last_verification": self.manifest.get("sources_unchanged") if self.manifest else None,
            "source_copy_count": len(self.manifest["copies"]) if self.manifest else 0,
            "official_executables_sha256": {name: sha256(path) for name, path in
                                           (("robot_mujoco", self.sim_binary), ("mc_ctrl", self.mc_binary)) if path.is_file()},
            "official_sources_modified": False,
        }

    def close(self):
        if self._closed:
            return
        errors, statuses = [], {}
        for name, process in reversed(list(self.processes.items())):
            initial_code = process.poll()
            termination_requested = forced_kill = False
            try:
                if initial_code is None:
                    termination_requested = True
                    process.terminate()
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        forced_kill = True
                        process.kill()
                        process.wait(timeout=2.0)
                statuses[name] = {"pid": process.pid, "exit_code": process.poll(),
                                  "exit_code_before_cleanup": initial_code,
                                  "termination_requested": termination_requested, "forced_kill": forced_kill}
            except (OSError, subprocess.TimeoutExpired) as exc:
                errors.append(f"Cannot stop owned {name} pid={process.pid}: {exc}")
        for handle in self._log_handles.values():
            handle.close()
        if self._global_lock is not None:
            self._global_lock.close()
            self._global_lock = None
        self._closed = True
        unchanged = self._verify_sources()
        if not unchanged:
            errors.append("Official source files changed while runtime was active")
        if self.workdir.is_dir():
            status = {"owned_processes": statuses, "cleanup_errors": errors, "source_files_unchanged": unchanged,
                      "termination_policy": "Only direct Popen-owned PIDs; no name/global process termination"}
            (self.workdir / "runtime_status.json").write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
        if errors:
            raise OfficialRuntimeError("; ".join(errors))
