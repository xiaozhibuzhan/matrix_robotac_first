"""Read-only SDK architecture checks, with no NumPy, simulator or SDK import."""
from __future__ import annotations

import hashlib
from pathlib import Path
import platform
import struct
import sys

ELF_MACHINES = {"x86_64": 62, "aarch64": 183}


def normalize_arch(machine=None):
    value = (platform.machine() if machine is None else machine).lower()
    value = {"amd64": "x86_64", "arm64": "aarch64"}.get(value, value)
    if value not in ELF_MACHINES:
        raise ValueError(f"Unsupported architecture {value!r}; use 64-bit x86_64 or aarch64 Linux")
    return value


def sdk_files(sdk_root, arch):
    arch = normalize_arch(arch)
    folder = Path(sdk_root).resolve()/"lib"/"zsl-1"/arch
    return {"library": folder/f"libmc_sdk_zsl_1_{arch}.so",
            "binding": folder/f"mc_sdk_zsl_1_py.cpython-310-{arch}-linux-gnu.so"}


def check_elf(path, arch):
    path = Path(path)
    with path.open("rb") as stream:
        header = stream.read(20)
    if (len(header) != 20 or header[:6] != b"\x7fELF\x02\x01"
            or struct.unpack("<H", header[18:20])[0] != ELF_MACHINES[normalize_arch(arch)]):
        raise ValueError(f"Expected 64-bit little-endian {arch} ELF: {path}")


def inspect_sdk(sdk_root):
    errors = []
    files, hashes = {}, {}
    try:
        arch = normalize_arch()
    except ValueError as exc:
        errors.append(str(exc))
        arch = None
    if (sys.platform != "linux" or platform.python_implementation() != "CPython"
            or sys.version_info[:2] != (3, 10) or struct.calcsize("P") != 8):
        errors.append("Official SDK runtime requires 64-bit Linux CPython 3.10")
    if arch:
        files = sdk_files(sdk_root, arch)
        for name, path in files.items():
            try:
                check_elf(path, arch)
                hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            except (OSError, ValueError) as exc:
                errors.append(str(exc))
    return {"ready": not errors, "errors": errors, "architecture": arch,
            "python": platform.python_version(), "platform": sys.platform,
            "sdk_root": str(Path(sdk_root).resolve()),
            "files": {name: str(path) for name, path in files.items()},
            "sha256": hashes, "sdk_imported": False, "sdk_initialized": False}
