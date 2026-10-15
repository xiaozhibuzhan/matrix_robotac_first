#!/usr/bin/env python3
"""Relocate five treasure root components in one exact cooked UE package.

Python 3.10+, standard library only. Never edits blueprint logic or map geometry.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import sys
import tempfile
from datetime import datetime, timezone


DEFAULT_LAYOUT = Path(__file__).with_name("treasure_layout_v1.json")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_layout(path):
    layout = json.loads(Path(path).read_text(encoding="utf-8"))
    if layout["schema_version"] != 1 or len(layout["boxes"]) != 5:
        raise ValueError("Unsupported layout schema or treasure count")
    spans = set()
    for box in layout["boxes"]:
        old, new = box["original_ue_cm"], box["patched_ue_cm"]
        if len(old) != 3 or len(new) != 3 or not all(map(math.isfinite, old + new)):
            raise ValueError("Invalid XYZ coordinates")
        if old[2] != new[2]:
            raise ValueError("Changing treasure height is not allowed")
        if list(new[:2]) != [box["map_center_m"][0] * 100,
                              -box["map_center_m"][1] * 100]:
            raise ValueError("UE/map coordinate conversion mismatch")
        span = set(range(box["xyz_offset"], box["xyz_offset"] + 16))
        if span & spans:
            raise ValueError("Overlapping patch ranges")
        spans.update(span)
    return layout


def package_exports(umap, layout):
    """Read names and fixed-size export records of the hash-verified package."""
    tables = layout["package_tables"]
    pos, names = tables["name_offset"], []
    for _ in range(tables["name_count"]):
        length = struct.unpack_from("<i", umap, pos)[0]
        pos += 4
        size = abs(length) * (2 if length < 0 else 1)
        raw = umap[pos:pos + size]
        name = raw.decode("utf-16-le" if length < 0 else "utf-8").rstrip("\0")
        names.append(name)
        pos += size + 4  # Two 16-bit name hashes.
    exports = []
    for index in range(tables["export_count"]):
        offset = tables["export_offset"] + index * tables["export_stride"]
        cls, _, _, outer, name_id, number, _ = struct.unpack_from("<7i", umap, offset)
        size, serial = struct.unpack_from("<qq", umap, offset + 28)
        name = names[name_id] + (f"_{number - 1}" if number else "")
        exports.append({"name": name, "class": cls, "outer": outer,
                        "size": size, "offset": serial - len(umap)})
    return exports


def inspect(umap, uexp, layout):
    if len(umap) != layout["umap_size"] or sha256(umap) != layout["umap_sha256"]:
        raise ValueError("Unsupported .umap version; no file was modified")
    if len(uexp) != layout["uexp_size"]:
        raise ValueError("Unsupported .uexp size; no file was modified")
    digest = sha256(uexp)
    if digest == layout["original_uexp_sha256"]:
        state, coordinate_key = "ORIGINAL", "original_ue_cm"
    elif digest == layout["patched_uexp_sha256"]:
        state, coordinate_key = "PATCHED", "patched_ue_cm"
    else:
        raise ValueError("Unsupported .uexp hash; no file was modified")
    exports = package_exports(umap, layout)
    actual = []
    for box in layout["boxes"]:
        actor = exports[box["actor_export"]]
        root = exports[box["root_export"]]
        if (actor["name"] != box["actor"] or actor["class"] != -3
                or actor["outer"] != 132 or root["name"] != "cash_lux"
                or root["class"] != -46 or root["outer"] != box["actor_export"] + 1
                or root["size"] != 83 or root["offset"] + 9 != box["xyz_offset"]):
            raise ValueError("Treasure export identity mismatch")
        if uexp[root["offset"]:box["xyz_offset"]].hex() != "7f0024029a04070301":
            raise ValueError("Treasure transform prefix mismatch")
        xyz = list(struct.unpack_from("<ddd", uexp, box["xyz_offset"]))
        if xyz != box[coordinate_key]:
            raise ValueError("Treasure transform mismatch")
        actual.append({"actor": box["actor"], "ue_cm": xyz,
                       "map_m": [xyz[0] / 100, -xyz[1] / 100, xyz[2] / 100]})
    return {"state": state, "umap_sha256": sha256(umap),
            "uexp_sha256": digest, "boxes": actual}


def make_patch(umap, original, layout):
    if inspect(umap, original, layout)["state"] != "ORIGINAL":
        raise ValueError("Patch source must be original")
    result = bytearray(original)
    allowed = set()
    for box in layout["boxes"]:
        offset = box["xyz_offset"]
        struct.pack_into("<dd", result, offset, *box["patched_ue_cm"][:2])
        allowed.update(range(offset, offset + 16))
    changed = {i for i, (before, after) in enumerate(zip(original, result)) if before != after}
    if len(result) != len(original) or not changed or changed - allowed:
        raise ValueError("Patch affected bytes outside the five XY transforms")
    result = bytes(result)
    if inspect(umap, result, layout)["state"] != "PATCHED":
        raise ValueError("Patch output validation failed")
    return result


def ensure_simulator_stopped():
    # Linux packaged runtime is the deployment target. No process is killed.
    if not sys.platform.startswith("linux"):
        return
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        executable = Path(os.fsdecode(raw[0])).name if raw and raw[0] else ""
        if executable in ("UeSim", "UnrealGame") or executable.startswith(("UeSim-", "UnrealGame-")):
            raise ValueError(f"UE simulator still running (PID {entry.name}); close it first")


def atomic_write(path, data, mode=None):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        if mode is not None:
            os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def backup_originals(backup, files, originals):
    backup.mkdir(parents=True, exist_ok=True)
    # Validate all existing backup files before writing any new one.
    for source, data in zip(files, originals):
        destination = backup / source.name
        if destination.exists() and destination.read_bytes() != data:
            raise ValueError(f"Backup differs from original: {destination}")
    for source, data in zip(files, originals):
        destination = backup / source.name
        if not destination.exists():
            with destination.open("xb") as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
        if destination.read_bytes() != data:
            raise ValueError(f"Backup verification failed: {destination}")


def operate(project, action, layout):
    project = Path(project).resolve(strict=True)
    relative = Path(layout["relative_stem"])
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("Unsafe resource path")
    stem = project / relative
    files = [stem.with_suffix(".umap"), stem.with_suffix(".uexp")]
    for path in files:
        if not path.resolve(strict=True).is_relative_to(project):
            raise ValueError("Resource resolves outside project")
    umap, current = (path.read_bytes() for path in files)
    report = inspect(umap, current, layout)
    backup = project / "scene_tools" / "backups" / layout["layout_id"]
    report.update({"project": str(project), "backup": str(backup), "action": action})
    if action == "status":
        return report
    ensure_simulator_stopped()
    mode = stat.S_IMODE(files[1].stat().st_mode)
    if action == "apply":
        if report["state"] == "PATCHED":
            # Reconstruct and verify original even when the package was copied
            # from another computer with the patch already applied.
            original = bytearray(current)
            for box in layout["boxes"]:
                struct.pack_into("<dd", original, box["xyz_offset"], *box["original_ue_cm"][:2])
            original = bytes(original)
            inspect(umap, original, layout)
        else:
            original = current
        updated = make_patch(umap, original, layout)
        backup_originals(backup, files, (umap, original))
        if report["state"] != "PATCHED":
            if files[0].read_bytes() != umap or files[1].read_bytes() != current:
                raise ValueError("Resource changed during preparation; retry after stopping UE")
            atomic_write(files[1], updated, mode)
    elif action == "restore":
        if report["state"] == "ORIGINAL":
            report["note"] = "Already original; no resource was modified"
            return report
        original_umap = (backup / files[0].name).read_bytes()
        original = (backup / files[1].name).read_bytes()
        if inspect(original_umap, original, layout)["state"] != "ORIGINAL":
            raise ValueError("Backup is not original")
        if files[0].read_bytes() != umap or files[1].read_bytes() != current:
            raise ValueError("Resource changed during restore preparation")
        atomic_write(files[1], original, mode)
    else:
        raise ValueError("Unknown operation")
    report.update(inspect(files[0].read_bytes(), files[1].read_bytes(), layout))
    report["utc"] = datetime.now(timezone.utc).isoformat()
    # Diagnostic log failure must not turn a completed resource change into
    # a misleading 'apply failed' result. Backups are verified before writing.
    try:
        atomic_write(backup / "last_operation.json",
                     (json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    except OSError as error:
        report["audit_log_warning"] = str(error)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "apply", "restore"))
    parser.add_argument("--project", type=Path, required=True,
                        help="Actual project containing run_sim.sh and src/UeSim")
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    args = parser.parse_args(argv)
    try:
        result = operate(args.project, args.action, read_layout(args.layout))
    except (OSError, ValueError, KeyError, IndexError, struct.error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
