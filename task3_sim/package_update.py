"""Package only Task 3 owned source, without environments, local settings or models."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import zipfile

EXCLUDED_DIRS = {".venv", ".venv_hardware", "runs", "runs_ubuntu", "updates", "__pycache__", ".pytest_cache", "local", ".git"}
SOURCE_SUFFIXES = {".py", ".json", ".md", ".txt", ".sh", ".png", ".xml", ".svg", ".jpg", ".jpeg", ".cpp", ".h", ".cmake"}


def source_files(source: Path) -> list[Path]:
    selected = []
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if any(part in EXCLUDED_DIRS for part in relative.parts):
            continue
        if not path.is_file() or (path.suffix.lower() not in SOURCE_SUFFIXES and path.name != ".gitignore"):
            continue
        if path.name == "PACKAGE_MANIFEST.json" or path.name.endswith("_local.json"):
            continue
        if path.is_symlink() or not path.resolve().is_relative_to(source.resolve()):
            raise ValueError(f"Source links are not packaged: {path}")
        selected.append(path)
    return selected


def make_package(source: Path, output: Path) -> dict:
    source = source.resolve()
    selected = source_files(source)
    if not selected:
        raise ValueError("No Task 3 source files to package")
    contents = {}
    for path in selected:
        data = path.read_bytes()
        # Windows source checkouts may acquire CRLF; Bash scripts in a Linux
        # update must retain Unix line endings without modifying the checkout.
        if path.suffix.lower() == ".sh":
            data = data.replace(b"\r\n", b"\n")
        contents[f"task3_sim/{path.relative_to(source).as_posix()}"] = data
    files = {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()}
    manifest = {
        "format_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Only task3_sim owned source; original model/SDK/UE/runtime are not included.",
        "excluded_directories": sorted(EXCLUDED_DIRS), "files": files,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name, data in contents.items():
            archive.writestr(name, data)
        archive.writestr("task3_sim/PACKAGE_MANIFEST.json", json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    # GNU sha256sum interprets CR from a Windows CRLF ending as part of the
    # filename. Bytes deliberately avoid platform newline translation.
    output.with_suffix(output.suffix + ".sha256").write_bytes(f"{digest}  {output.name}\n".encode("ascii"))
    return {"archive": str(output.resolve()), "source_files": len(files), "sha256": digest}


def main(argv: list[str] | None = None) -> int:
    source = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=source / "updates" / "task3_arm64_038_20261022.zip")
    args = parser.parse_args(argv)
    print(json.dumps(make_package(source, args.output.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
