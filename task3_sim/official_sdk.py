"""Strict HighLevel adapter for the unmodified official ZSL-1 SDK.

The API spellings come from ``deps/zsibot_sdk/include/zsl-1/highlevel.h``;
the ranges and state requirements come from ``docs/api_zsl-1.md``.  The
official demo waits four seconds after standUp.  move() and attitudeControl()
are separate modes. Fine yaw and pitch can share attitudeControl, while
translation and body attitude are never sent as simultaneous command modes.

Importing this module never loads a native library or connects to a robot.
Native loading supports Linux CPython 3.10 only.  Both default endpoints are
loopback.  An injected SDK, clock and wait function support deterministic
physics-bridge tests without pretending that the proprietary controller ran.

The SDK getters expose no acquisition time.  A caller must supply trusted
FeedbackStamp metadata from the receiving bridge, in the session clock domain,
to make feedback valid.  Getter read time alone is never a sensor timestamp.
The optional watchdog thread handles a stalled Python caller, not a killed
process or a blocked native call; real deployment also needs an independent
process/robot command timeout.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import importlib
import ipaddress
import math
import operator
from pathlib import Path
import platform
import sys
import threading
import time
from typing import Callable

import numpy as np

from .interfaces import RobotFeedback, RobotMotionCommand


class OfficialSDKError(RuntimeError):
    """Rejected SDK action, disconnected session, or unverified feedback."""


class SDKWatchdogError(OfficialSDKError):
    """Latched command timeout; a new session is required."""


@dataclass(frozen=True)
class FeedbackStamp:
    sequence: int
    source_timestamp_s: float | None
    received_at_s: float


@dataclass(frozen=True)
class SDKLimits:
    # Conservative task-level limits, never wider than the official API.
    max_vx_m_s: float = 0.15
    max_vy_m_s: float = 0.15
    max_yaw_rate_rad_s: float = 0.35
    max_pitch_rate_rad_s: float = 0.18
    max_attitude_yaw_rate_rad_s: float = 0.35


@dataclass(frozen=True)
class _CommandLease:
    family: str
    command: RobotMotionCommand
    issued_at_s: float
    valid_until_s: float | None


def load_official_sdk(sdk_root: str | Path | None = None):
    """Load the supplied binary only on its declared Python/platform ABI."""
    if sys.platform != "linux" or platform.python_implementation() != "CPython" or sys.version_info[:2] != (3, 10):
        raise OfficialSDKError("Official SDK runtime requires Linux CPython 3.10")
    arch = {"amd64": "x86_64", "arm64": "aarch64"}.get(platform.machine().lower(), platform.machine().lower())
    if arch not in ("x86_64", "aarch64"):
        raise OfficialSDKError(f"Unsupported official SDK architecture: {arch}")
    root = Path(sdk_root) if sdk_root is not None else Path(__file__).resolve().parents[1] / "deps" / "zsibot_sdk"
    directory = (root / "lib" / "zsl-1" / arch).resolve()
    libraries = list(directory.glob("mc_sdk_zsl_1_py.cpython-310-*.so"))
    native = directory / f"libmc_sdk_zsl_1_{arch}.so"
    if len(libraries) != 1 or not native.is_file():
        raise OfficialSDKError(f"Matching official SDK binaries missing from {directory}")
    loaded = sys.modules.get("mc_sdk_zsl_1_py")
    if loaded is not None and Path(getattr(loaded, "__file__", "")).resolve() != libraries[0].resolve():
        raise OfficialSDKError("A different mc_sdk_zsl_1_py is already loaded")
    ctypes.CDLL(str(native), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(directory))
    try:
        module = importlib.import_module("mc_sdk_zsl_1_py")
    finally:
        sys.path.remove(str(directory))
    if Path(module.__file__).resolve() != libraries[0].resolve():
        raise OfficialSDKError("Imported SDK is outside the explicitly selected official SDK root")
    return module.HighLevel()


class OfficialSDKSession:
    """RobotInterface with checked returns, mode transitions and source freshness.

    ``wait(seconds)`` must advance ``clock()``.  A physics bridge can use it to
    step MuJoCo while the SDK state machine waits.  ``freshness_provider`` must
    describe the actual getter sample, not manufacture a heartbeat per read.
    It may supply source_timestamp_s=None to report receipts honestly; those
    readings stay invalid for motion and shooting until acquisition is known.
    Alternatively ``state_feedback_provider`` supplies actual acquired official
    RobotState fields as RobotFeedback. In that case SDK getter values are only
    diagnostics; their read time is never replaced with the RobotState time.
    ``allow_void_status`` is an explicit compatibility exception for a binding
    that demonstrably discards the C++ uint32 return; it is recorded in events.
    ExecuteMove/ExecuteAttitude return the applied command. A mode switch
    completes a stopped standUp request, confirms it through the same public
    API after fresh standing feedback, waits four seconds and returns zero: the caller must
    reacquire an image and issue a new command after that four-second wait.
    Optional refresh_interval_s repeats exactly the last payload while its
    original issued_at_s and optional valid_until_s remain valid. Refreshing
    never creates a new observation or extends either deadline. Source state
    loss or image-lease expiry cancels refresh with an owned-family zero;
    original command timeout, disconnect and SDK failures still latch a fault.
    """

    def __init__(self, *, sdk=None, sdk_root=None, local_ip="127.0.0.1", local_port=43988,
                 robot_ip="127.0.0.1", clock: Callable[[], float] = time.monotonic,
                 wait: Callable[[float], None] = time.sleep, freshness_provider=None,
                 state_feedback_provider=None,
                 limits: SDKLimits | None = None, command_timeout_s=0.25,
                 max_feedback_age_s=0.15, stationary_duration_s=0.2,
                 max_stationary_speed_m_s=0.01, max_stationary_gyro_rad_s=0.02,
                 stand_up_min_s=4.0, transition_timeout_s=12.0, connect_timeout_s=10.0,
                 poll_s=0.025, allow_void_status=False, background_watchdog=True,
                 refresh_interval_s=None):
        self.sdk = sdk
        self.sdk_root = sdk_root
        self.local_ip, self.robot_ip = str(ipaddress.ip_address(local_ip)), str(ipaddress.ip_address(robot_ip))
        if not isinstance(local_port, int) or isinstance(local_port, bool) or not 1 <= local_port <= 65535:
            raise ValueError("local_port must be an integer from 1 to 65535")
        self.local_port = local_port
        self.clock, self.wait, self.freshness_provider = clock, wait, freshness_provider
        self.state_feedback_provider = state_feedback_provider
        self.limits = limits or SDKLimits()
        maxima = (self.limits.max_vx_m_s, self.limits.max_vy_m_s,
                  self.limits.max_yaw_rate_rad_s, self.limits.max_pitch_rate_rad_s,
                  self.limits.max_attitude_yaw_rate_rad_s)
        if not all(math.isfinite(x) and 0 < x <= upper for x, upper in zip(maxima, (3.0, 1.0, 3.0, 0.5, 0.5))):
            raise ValueError("Task limits must be positive and within official move/attitude ranges")
        timing = (command_timeout_s, max_feedback_age_s, stationary_duration_s,
                  max_stationary_speed_m_s, max_stationary_gyro_rad_s,
                  stand_up_min_s, transition_timeout_s, connect_timeout_s, poll_s)
        if not all(math.isfinite(x) and x > 0 for x in timing):
            raise ValueError("Timing and stationary thresholds must be positive finite values")
        if stand_up_min_s < 4 or transition_timeout_s <= stand_up_min_s or poll_s > command_timeout_s / 2:
            raise ValueError("StandUp needs >=4 s, a longer transition timeout, and watchdog polling <=timeout/2")
        if refresh_interval_s is not None and (isinstance(refresh_interval_s, bool)
                or not math.isfinite(refresh_interval_s) or refresh_interval_s < 0.002
                or refresh_interval_s > command_timeout_s):
            raise ValueError("refresh_interval_s must be finite, >=0.002 s (official 500 Hz maximum), and within the command timeout")
        self.command_timeout_s, self.max_feedback_age_s = command_timeout_s, max_feedback_age_s
        self.stationary_duration_s = stationary_duration_s
        self.max_stationary_speed_m_s = max_stationary_speed_m_s
        self.max_stationary_gyro_rad_s = max_stationary_gyro_rad_s
        self.stand_up_min_s, self.transition_timeout_s = stand_up_min_s, transition_timeout_s
        self.connect_timeout_s, self.poll_s = connect_timeout_s, poll_s
        self.allow_void_status, self.background_watchdog = allow_void_status, background_watchdog
        self.refresh_interval_s = refresh_interval_s
        self.connected, self.standing, self.closed = False, False, False
        self.mode = "disconnected"
        self.events = []
        self.last_feedback_meta = {}
        self.last_command = RobotMotionCommand()
        self.last_command_at_s = None
        self.fault = None
        self._command_mode = None
        self._command_lease = None
        self._last_transmit_at_s = None
        self._last_stamp = None
        self._last_source_timestamp = -math.inf
        self._lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = None

    def _event(self, kind, **fields):
        self.events.append({"read_at_s": float(self.clock()), "kind": kind, **fields})

    def _live(self):
        if self.closed:
            raise OfficialSDKError("SDK session is closed")
        if self.fault is not None:
            raise self.fault
        if not self.connected:
            raise OfficialSDKError("Connect must precede this action")

    def _status(self, operation, *args, command_context=None):
        context = {**(command_context or {}), "operation": operation, "arguments": list(args),
                   "sdk_called_at_s": float(self.clock())}
        try:
            result = getattr(self.sdk, operation)(*args)
        except Exception as exc:
            self._event("sdk_call_failed", **context, error=str(exc))
            raise
        if result is None and self.allow_void_status:
            self._event("sdk_status_unavailable", **{**context,
                        "explanation": "explicitly enabled void-return binding compatibility; feedback still required"})
            return
        try:
            code = operator.index(result)
        except TypeError as exc:
            self._event("sdk_status_invalid", **context, result=repr(result))
            raise OfficialSDKError(f"{operation}: expected official uint32 status, got {result!r}") from exc
        if isinstance(result, bool) or not 0 <= code <= 0xFFFFFFFF:
            self._event("sdk_status_invalid", **context, result=repr(result))
            raise OfficialSDKError(f"{operation}: invalid uint32 status {result!r}")
        self._event("sdk_result", **context, code=code)
        if code:
            raise OfficialSDKError(f"{operation}: official SDK returned {code} (0x{code:04X})")

    def _zero(self):
        # A zero move is still a mode-changing SDK action. Never send it before
        # a successful standUp, or use it to stop an attitude action.
        lease = self._command_lease
        self._command_lease = None
        self._last_transmit_at_s = None
        context = self._lease_context(lease, refresh=False, safety_zero=True) if lease else None
        if self._command_mode == "move":
            self._status("move", 0.0, 0.0, 0.0, command_context=context)
        elif self._command_mode == "attitude":
            self._status("attitudeControl", 0.0, 0.0, 0.0, 0.0, command_context=context)
        self.last_command = RobotMotionCommand()
        self.last_command_at_s = float(self.clock())

    @staticmethod
    def _lease_context(lease, **fields):
        command = lease.command
        arguments = ([0.0, command.pitch_rate_rad_s, command.yaw_rate_rad_s, 0.0]
                     if lease.family == "attitude"
                     else [command.vx_m_s, command.vy_m_s, command.yaw_rate_rad_s])
        return {"command_family": lease.family, "original_issued_at_s": lease.issued_at_s,
                "valid_until_s": lease.valid_until_s,
                "operation": "attitudeControl" if lease.family == "attitude" else "move",
                "arguments": arguments, **fields}

    def _send_leased_command(self, lease, *, refresh):
        command = lease.command
        context = self._lease_context(lease, refresh=refresh)
        if lease.family == "attitude":
            self._status("attitudeControl", 0.0, command.pitch_rate_rad_s,
                         command.yaw_rate_rad_s, 0.0, command_context=context)
        else:
            self._status("move", command.vx_m_s, command.vy_m_s,
                         command.yaw_rate_rad_s, command_context=context)
        self._last_transmit_at_s = float(self.clock())

    def _expire_lease(self, lease, now):
        if lease.valid_until_s is not None and now >= lease.valid_until_s:
            self._zero()
            self._event("command_lease_expired", **self._lease_context(lease))
            return True
        return False

    def _trip(self, exc):
        if self.fault is None:
            self.fault = exc if isinstance(exc, OfficialSDKError) else OfficialSDKError(str(exc))
            self._event("fault", error=str(self.fault))
        try:
            self._zero()
        except Exception as stop_error:
            self._event("stop_failed", error=str(stop_error))
            self.fault.add_note(f"Current-mode zero command also failed: {stop_error}") if hasattr(self.fault, "add_note") else None
        self.mode = "fault"
        raise self.fault

    def Connect(self):
        with self._lock:
            if self.closed or self.fault is not None:
                raise OfficialSDKError("Cannot reconnect a closed or faulted session; create a new session")
            if self.connected:
                return
            if self.sdk is None:
                self.sdk = load_official_sdk(self.sdk_root)
            for name in ("initRobot", "checkConnect", "standUp", "move", "attitudeControl", "getRPY",
                         "getPosition", "getBodyGyro", "getWorldVelocity", "getCurrentCtrlmode"):
                if not callable(getattr(self.sdk, name, None)):
                    raise OfficialSDKError(f"Official SDK binding is missing {name}")
            self.sdk.initRobot(self.local_ip, self.local_port, self.robot_ip)
            deadline = float(self.clock()) + self.connect_timeout_s
            while not self.sdk.checkConnect():
                if float(self.clock()) >= deadline:
                    raise OfficialSDKError("Official SDK connection timeout")
                self.wait(self.poll_s)
            self.connected, self.mode = True, "connected"
            self._event("connected", local_ip=self.local_ip, robot_ip=self.robot_ip, local_port=self.local_port)
        if self.background_watchdog:
            self.start_watchdog()

    def _read_feedback(self):
        now = float(self.clock())
        values = [np.full(3, np.nan) for _ in range(4)]
        stamp = None
        source_feedback = None
        problem = None
        sdk_error = None
        try:
            if not self.connected or self.closed or self.fault is not None or not self.sdk.checkConnect():
                raise OfficialSDKError("SDK feedback connection unavailable")
            for index, name in enumerate(("getRPY", "getPosition", "getBodyGyro", "getWorldVelocity")):
                try:
                    raw = getattr(self.sdk, name)()
                except Exception as exc:
                    sdk_error = f"{name} failed: {exc}"
                    raise OfficialSDKError(sdk_error) from exc
                value = np.asarray(raw, dtype=float)
                if value.shape != (3,) or not np.all(np.isfinite(value)):
                    raise OfficialSDKError(f"{name}: expected three finite values")
                values[index] = value.copy()
            if self.state_feedback_provider is not None:
                source_feedback = self.state_feedback_provider()
                now = float(self.clock())
                if not isinstance(source_feedback, RobotFeedback) or not source_feedback.valid:
                    raise OfficialSDKError("Official RobotState feedback source is invalid")
                source_values = [np.asarray(x, dtype=float) for x in (source_feedback.rpy_rad, source_feedback.position_m,
                                                                     source_feedback.gyro_rad_s, source_feedback.velocity_m_s)]
                source_time = float(source_feedback.timestamp)
                if (not all(x.shape == (3,) and np.all(np.isfinite(x)) for x in source_values)
                        or not math.isfinite(source_time) or not 0 <= now - source_time <= self.max_feedback_age_s
                        or source_time < self._last_source_timestamp):
                    raise OfficialSDKError("Official RobotState vectors/time are invalid, stale, or reversed")
                self._last_source_timestamp = source_time
                self.last_feedback_meta = {
                    "read_at_s": now, "source_timestamp_s": source_time, "received_at_s": None,
                    "sequence": source_time, "valid": True, "error": None,
                    "timestamp_policy": "external official RobotState acquisition; SDK getter reads have no source timestamp",
                    "sdk_getter_values": [x.tolist() for x in values], "sdk_getter_read_at_s": now,
                }
                return RobotFeedback(True, *(x.copy() for x in source_values), source_time)
            stamp = self.freshness_provider() if self.freshness_provider else None
            now = float(self.clock())
            if not isinstance(stamp, FeedbackStamp) or stamp.source_timestamp_s is None:
                raise OfficialSDKError("SDK getters have no source timestamp; a trusted acquisition heartbeat is required")
            if (not isinstance(stamp.sequence, int) or isinstance(stamp.sequence, bool) or stamp.sequence < 0
                    or not all(math.isfinite(x) for x in (now, stamp.source_timestamp_s, stamp.received_at_s))
                    or not 0 <= now - stamp.received_at_s <= self.max_feedback_age_s
                    or not 0 <= now - stamp.source_timestamp_s <= self.max_feedback_age_s
                    or stamp.source_timestamp_s > stamp.received_at_s):
                raise OfficialSDKError("SDK feedback heartbeat is invalid, stale, future, or uses a different clock domain")
            previous = self._last_stamp
            if previous is not None and (stamp.sequence < previous.sequence
                    or (stamp.sequence == previous.sequence and stamp.source_timestamp_s != previous.source_timestamp_s)
                    or (stamp.sequence > previous.sequence and stamp.source_timestamp_s <= previous.source_timestamp_s)):
                raise OfficialSDKError("SDK feedback heartbeat sequence/acquisition time reversed or was restamped")
            self._last_stamp = stamp
        except Exception as exc:
            problem = str(exc)
        valid = problem is None
        self.last_feedback_meta = {"read_at_s": now, "source_timestamp_s": getattr(stamp, "source_timestamp_s", None),
                                   "received_at_s": getattr(stamp, "received_at_s", None),
                                   "sequence": getattr(stamp, "sequence", None), "valid": valid, "error": problem,
                                   "sdk_error": sdk_error,
                                   "timestamp_policy": "trusted source acquisition time; getter read time is diagnostic only"}
        timestamp = float(stamp.source_timestamp_s) if valid else -math.inf
        return RobotFeedback(valid, *values, timestamp)

    def ReadFeedback(self):
        with self._lock:
            return self._read_feedback()

    def _wait_stationary(self, *, min_until=None, require_stand_mode=False, deadline=None):
        if deadline is None:
            deadline = float(self.clock()) + self.transition_timeout_s
        stable_since = None
        last_sequence = None
        while True:
            now = float(self.clock())
            if not self.sdk.checkConnect():
                raise OfficialSDKError("SDK disconnected during state transition")
            feedback = self._read_feedback()
            ctrlmode = self.sdk.getCurrentCtrlmode()
            now = float(self.clock())
            if now >= deadline:
                raise OfficialSDKError(f"Timed out waiting for fresh stationary feedback / standing mode: {self.last_feedback_meta.get('error')}")
            stationary = (feedback.valid and 0 <= now-feedback.timestamp <= self.max_feedback_age_s
                          and np.linalg.norm(feedback.gyro_rad_s) <= self.max_stationary_gyro_rad_s
                          and np.linalg.norm(feedback.velocity_m_s) <= self.max_stationary_speed_m_s
                          and (not require_stand_mode or ctrlmode == 1))
            sequence = self.last_feedback_meta.get("sequence")
            if not stationary:
                stable_since = None
            elif sequence != last_sequence:
                if stable_since is None:
                    stable_since = now
                last_sequence = sequence
                if now - stable_since >= self.stationary_duration_s and (min_until is None or now >= min_until):
                    self._event("stationary_confirmed", control_mode=int(ctrlmode), source_timestamp_s=feedback.timestamp)
                    return
            if now >= deadline:
                raise OfficialSDKError(f"Timed out waiting for fresh stationary feedback / standing mode: {self.last_feedback_meta.get('error')}")
            before = now
            self.wait(self.poll_s)
            if not float(self.clock()) > before:
                raise OfficialSDKError("Injected wait must advance the session clock")

    def StandUp(self):
        with self._lock:
            self._live()
            self.standing = False
            try:
                deadline = float(self.clock()) + self.transition_timeout_s
                if self._command_mode is not None:
                    self._zero()
                    self._wait_stationary(deadline=deadline)
                else:
                    # Do not standUp over locomotion started by another owner,
                    # or assume an unspecified action mode is body attitude.
                    initial_mode = self.sdk.getCurrentCtrlmode()
                    if isinstance(initial_mode, bool) or initial_mode not in (0, 1, 10, 51):
                        raise OfficialSDKError(f"Refusing standUp from unowned SDK mode {initial_mode!r}")
                    self._wait_stationary(deadline=deadline)
                self._status("standUp")
                self.mode, self._command_mode, self.standing = "standing", None, False
                self._event("standup_requested")
                # The supplied SDK counts standUp requests made outside mode 1
                # across calls, even when they return 0. Only a call *in* mode 1
                # clears that counter (see validation/standup036_sdk_evidence.json).
                # Wait for acquired stationary state AND the SDK's standing mode
                # before confirming through its public API. Never retry a failure.
                self._wait_stationary(require_stand_mode=True, deadline=deadline)
                if self.sdk.getCurrentCtrlmode() != 1:
                    raise OfficialSDKError("Standing mode lost before standUp acknowledgement")
                if float(self.clock()) >= deadline:
                    raise OfficialSDKError("StandUp transition deadline expired before acknowledgement")
                source_time = self.last_feedback_meta.get("source_timestamp_s")
                if (source_time is None
                        or not 0 <= float(self.clock())-source_time <= self.max_feedback_age_s):
                    raise OfficialSDKError("Standing feedback expired before standUp acknowledgement")
                self._status("standUp")
                self._event("standup_acknowledged", control_mode=1,
                            source_timestamp_s=self.last_feedback_meta.get("source_timestamp_s"))
                # This checked acknowledgement is the last SDK standUp call.
                # Retain the full official-demo wait after it, not before it.
                min_until = float(self.clock()) + self.stand_up_min_s
                self._wait_stationary(min_until=min_until, require_stand_mode=True, deadline=deadline)
                self.standing = True
            except Exception as exc:
                self._trip(exc)

    def map_command(self, command, *, mode=None):
        if mode not in (None, "move", "attitude"):
            raise ValueError("Command mode must be move or attitude")
        requested = tuple(float(x) for x in (command.vx_m_s, command.vy_m_s,
                                             command.yaw_rate_rad_s, command.pitch_rate_rad_s))
        if not all(math.isfinite(x) for x in requested):
            raise OfficialSDKError("Nonfinite motion command")
        if mode == "attitude":
            if any(requested[:2]):
                raise OfficialSDKError("attitudeControl cannot include translational move velocity")
            maxima = (self.limits.max_vx_m_s, self.limits.max_vy_m_s,
                      self.limits.max_attitude_yaw_rate_rad_s, self.limits.max_pitch_rate_rad_s)
        else:
            maxima = (self.limits.max_vx_m_s, self.limits.max_vy_m_s,
                      self.limits.max_yaw_rate_rad_s, self.limits.max_pitch_rate_rad_s)
        if any(abs(value) > maximum for value, maximum in zip(requested, maxima)):
            raise OfficialSDKError("Command exceeds configured / official SDK limits; command is rejected, never clipped")
        if mode == "move" and requested[3] != 0:
            raise OfficialSDKError("ExecuteMove cannot include pitch; use ExecuteAttitude after a stopped transition")
        if mode is None and requested[3] != 0 and any(requested[:3]):
            raise OfficialSDKError("move and pitch attitudeControl cannot be mixed; use separate stopped stages")
        mapped = list(requested)
        for index, minimum in enumerate((0.05, 0.1, 0.02) if mode != "attitude" else ()):
            if 0 < abs(mapped[index]) < minimum:
                self._event("deadband_zero", axis=("vx", "vy", "yaw")[index], requested=mapped[index], sent=0.0,
                            reason="official nonzero move range; never increase a small request")
                mapped[index] = 0.0
        return RobotMotionCommand(*mapped)

    def Execute(self, command, *, issued_at_s=None):
        """Compatibility dispatcher; ambiguous mixed yaw and pitch is rejected."""
        return self._execute(command, issued_at_s=issued_at_s)

    def ExecuteMove(self, command, *, issued_at_s=None, valid_until_s=None):
        """Coarse locomotion through official move(vx, vy, yaw_rate)."""
        return self._execute(command, requested_mode="move", issued_at_s=issued_at_s,
                             valid_until_s=valid_until_s)

    def ExecuteAttitude(self, command, *, issued_at_s=None, valid_until_s=None):
        """Fine stationary aiming; official attitudeControl(0, pitch, yaw, 0)."""
        return self._execute(command, requested_mode="attitude", issued_at_s=issued_at_s,
                             valid_until_s=valid_until_s)

    def _execute(self, command, *, requested_mode=None, issued_at_s=None, valid_until_s=None):
        with self._lock:
            self._live()
            try:
                self.poll_watchdog()
                # A newly acquired image cannot erase the evidence that the
                # previous image lease already expired between monitor polls.
                # Stop its owned family before considering the replacement.
                previous = self._command_lease
                if previous is not None:
                    self._expire_lease(previous, float(self.clock()))
                if not self.standing:
                    raise OfficialSDKError("A completed StandUp is required before move or attitudeControl")
                now = float(self.clock())
                issued = now if issued_at_s is None else float(issued_at_s)
                if not math.isfinite(issued) or not 0 <= now - issued <= self.command_timeout_s:
                    raise SDKWatchdogError("Expired or future issued motion command")
                until = None if valid_until_s is None else float(valid_until_s)
                if until is not None and not math.isfinite(until):
                    raise OfficialSDKError("Command lease deadline must be finite")
                mapped = self.map_command(command, mode=requested_mode)
                if not self.sdk.checkConnect():
                    raise OfficialSDKError("Official SDK disconnected")
                nonzero = any((mapped.vx_m_s, mapped.vy_m_s, mapped.yaw_rate_rad_s, mapped.pitch_rate_rad_s))
                if not nonzero:
                    # A safety WAIT must still stop an owned command family
                    # when its sensor sample has expired. _zero never enters a
                    # new family (and sends no SDK action while standing), so it needs
                    # no fresh motion feedback. Connection, standing, command
                    # validity and any latched watchdog remain checked above.
                    self._zero()
                    return mapped
                requested_mode = requested_mode or ("attitude" if mapped.pitch_rate_rad_s else "move")
                lease = _CommandLease(requested_mode, mapped, issued, until)
                if self._expire_lease(lease, now):
                    return RobotMotionCommand()
                feedback = self._read_feedback()
                if not feedback.valid:
                    raise OfficialSDKError(f"Motion requires verified fresh feedback: {self.last_feedback_meta['error']}")
                # Freshness reads may block; neither original deadline can be
                # renewed by reading feedback or by retransmitting its command.
                self.poll_watchdog()
                previous = self._command_lease
                if previous is not None:
                    self._expire_lease(previous, float(self.clock()))
                if self._command_mode is not None and self._command_mode != requested_mode:
                    self.StandUp()
                    self._event("mode_prepared_waiting_new_command", target_mode=requested_mode,
                                reason="four-second transition invalidates the original image-derived command")
                    return RobotMotionCommand()
                now = float(self.clock())
                if not 0 <= now-issued <= self.command_timeout_s:
                    raise SDKWatchdogError("Original motion command expired while reading feedback")
                if self._expire_lease(lease, now):
                    return RobotMotionCommand()
                # Mark the family before calling so even a rejected action is
                # followed only by its matching zero command.
                self._command_mode = requested_mode
                self._command_lease = lease
                self._send_leased_command(lease, refresh=False)
                self.mode = requested_mode
                self.last_command = mapped
                self.last_command_at_s = issued
                self._event("command_lease_started", command=vars(mapped), **self._lease_context(lease))
                return mapped
            except Exception as exc:
                self._trip(exc)

    def Stop(self):
        with self._lock:
            if self.closed:
                return
            try:
                self._zero()
            except Exception as exc:
                self._trip(exc)

    def poll_watchdog(self):
        """Latch and send a current-mode zero after timeout or disconnect."""
        with self._lock:
            if self.fault is not None:
                raise self.fault
            if self.closed or not self.connected:
                return
            try:
                connection_ok = self.sdk.checkConnect()
            except Exception as exc:
                self._trip(OfficialSDKError(f"Official SDK connection check failed: {exc}"))
            if not connection_ok:
                self._trip(OfficialSDKError("Official SDK disconnected"))
            if self.last_command_at_s is not None and any((self.last_command.vx_m_s, self.last_command.vy_m_s,
                                                         self.last_command.yaw_rate_rad_s, self.last_command.pitch_rate_rad_s)):
                age = float(self.clock()) - self.last_command_at_s
                if age < 0 or age > self.command_timeout_s:
                    self._trip(SDKWatchdogError("Motion watchdog expired or session clock reversed; zero sent and session latched"))

    def check_current_lease(self):
        """Check current command authority without refreshing its payload.

        This is also usable while a caller holds ``_lock`` for a final shot
        gate. It rechecks the original watchdog and the image deadline after
        any potentially blocking SDK/feedback reads. Image expiry sends the
        owned-family zero and returns False; original timeout or connection
        failure still latches. A stationary zero command has no active lease.
        This check does not acquire new feedback or extend either deadline.
        """
        with self._lock:
            self.poll_watchdog()
            if self.closed or not self.connected:
                return False
            lease = self._command_lease
            if lease is None:
                return False
            try:
                return not self._expire_lease(lease, float(self.clock()))
            except Exception as exc:
                self._trip(exc)

    def poll_refresh(self):
        """Repeat a leased payload without renewing its original lifetime.

        This does not acquire a new image or create a new control decision.
        Expired image leases and missing fresh state stop the owned family;
        an expired original command, disconnect or SDK error still latches.
        """
        with self._lock:
            self.poll_watchdog()
            if self.closed or not self.connected:
                return
            lease = self._command_lease
            if lease is None:
                return
            try:
                now = float(self.clock())
                if self._expire_lease(lease, now):
                    return
                if self.refresh_interval_s is None:
                    return
                if (self._command_mode != lease.family or not self.standing):
                    self._zero()
                    self._event("command_refresh_cancelled", reason="command family or standing ownership changed",
                                **self._lease_context(lease))
                    return
                if now-self._last_transmit_at_s < self.refresh_interval_s:
                    return
                feedback = self._read_feedback()
                # A native getter may consume time or fail/disconnect. Recheck
                # original watchdog and lease before another SDK action.
                self.poll_watchdog()
                if self._expire_lease(lease, float(self.clock())):
                    return
                if self.last_feedback_meta.get("sdk_error"):
                    raise OfficialSDKError(self.last_feedback_meta["sdk_error"])
                if not feedback.valid:
                    self._zero()
                    self._event("command_refresh_cancelled", reason="fresh feedback unavailable",
                                feedback_error=self.last_feedback_meta.get("error"),
                                **self._lease_context(lease))
                    return
                self._send_leased_command(lease, refresh=True)
                self._event("command_refresh", command=vars(lease.command),
                            feedback_source_timestamp_s=feedback.timestamp, **self._lease_context(lease))
            except Exception as exc:
                self._trip(exc)

    def start_watchdog(self):
        with self._lock:
            if self._watchdog_thread is not None:
                return
            def monitor():
                while not self._watchdog_stop.wait(self.poll_s):
                    try:
                        self.poll_refresh()
                    except OfficialSDKError:
                        return  # fault and any failed zero are already retained
            self._watchdog_thread = threading.Thread(target=monitor, name="task3-sdk-watchdog", daemon=True)
            self._watchdog_thread.start()

    def close(self):
        self._watchdog_stop.set()
        try:
            self.Stop()
        finally:
            with self._lock:
                self.closed, self.connected, self.standing = True, False, False
                self.mode = "closed"
            if self._watchdog_thread is not None and self._watchdog_thread is not threading.current_thread():
                self._watchdog_thread.join(timeout=max(0.1, self.poll_s * 3))

