#!/usr/bin/env python3
"""Inspect or clean a separate copy of a mapper XYZ binary PCD file."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Sequence

import numpy as np


def read_pcd(path: Path) -> np.ndarray:
    """Read only unorganized binary PCD XYZ float32 data, with exact byte counts."""
    fields: dict[str, list[str]] = {}
    allowed = {"VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS", "DATA"}
    with path.open("rb") as source:
        header_bytes = 0
        while True:
            line = source.readline(65537)
            header_bytes += len(line)
            if not line or header_bytes > 65536:
                raise ValueError("PCD header is missing DATA or exceeds 64 KiB")
            try:
                tokens = line.decode("ascii").strip().split()
            except UnicodeDecodeError as exc:
                raise ValueError("PCD header must be ASCII") from exc
            if not tokens or tokens[0].startswith("#"):
                continue
            key, values = tokens[0], tokens[1:]
            if key not in allowed or key in fields:
                raise ValueError(f"Unknown or repeated PCD header field: {key}")
            fields[key] = values
            if key == "DATA":
                break

        expected = {"FIELDS": ["x", "y", "z"], "SIZE": ["4", "4", "4"],
                    "TYPE": ["F", "F", "F"], "COUNT": ["1", "1", "1"],
                    "HEIGHT": ["1"], "DATA": ["binary"]}
        if fields.get("VERSION") not in (["0.7"], [".7"]):
            raise ValueError("Only PCD VERSION 0.7 is supported")
        for key, value in expected.items():
            if fields.get(key) != value:
                raise ValueError(f"Expected PCD {key} {' '.join(value)}; found {fields.get(key)}")
        for key in ("WIDTH", "POINTS"):
            values = fields.get(key, [])
            if len(values) != 1 or not values[0].isascii() or not values[0].isdecimal():
                raise ValueError(f"PCD {key} must be one nonnegative integer")
        count = int(fields["POINTS"][0])
        if int(fields["WIDTH"][0]) != count:
            raise ValueError("PCD WIDTH must equal POINTS for HEIGHT 1")
        if "VIEWPOINT" in fields:
            try:
                viewpoint = [float(value) for value in fields["VIEWPOINT"]]
            except ValueError as exc:
                raise ValueError("Invalid PCD VIEWPOINT") from exc
            if len(viewpoint) != 7 or not all(math.isfinite(value) for value in viewpoint):
                raise ValueError("PCD VIEWPOINT must contain seven finite values")
        expected_bytes = count * 12
        remaining_bytes = os.fstat(source.fileno()).st_size - source.tell()
        if remaining_bytes != expected_bytes:
            raise ValueError(f"PCD payload length mismatch: expected {expected_bytes} bytes, found {remaining_bytes}")
        payload = source.read()
        if len(payload) != expected_bytes:
            raise ValueError("PCD file changed while being read")
    return np.frombuffer(payload, dtype="<f4").reshape(count, 3)


def point_summary(points: np.ndarray) -> dict:
    finite = points[np.isfinite(points).all(axis=1)]
    return {
        "points": len(points),
        "finite_points": len(finite),
        "nonfinite_points": len(points) - len(finite),
        "bounds": {"min": finite.min(axis=0).astype(float).tolist(),
                   "max": finite.max(axis=0).astype(float).tolist()} if len(finite) else None,
    }


def clean_points(points: np.ndarray, radius: float = 0.2, min_neighbors: int = 2,
                 bounds: Sequence[float] | None = None) -> tuple[np.ndarray, dict[str, int]]:
    try:
        from .point_filters import filter_xyz
    except ImportError:
        from point_filters import filter_xyz

    if bounds is not None:
        bounds_array = np.asarray(bounds, dtype=np.float64)
        if bounds_array.shape != (6,) or not np.isfinite(bounds_array).all():
            raise ValueError("Bounds must contain six finite numbers")
        lower, upper = bounds_array[::2], bounds_array[1::2]
        if np.any(lower > upper):
            raise ValueError("Each bounds minimum must be <= its maximum")

    # These are world coordinates. Range from (0, 0, 0) is not sensor range.
    finite, first_stats = filter_xyz(points, min_range=0, max_range=0, radius=0)
    bounded = finite
    if bounds is not None:
        bounded = finite[np.logical_and(finite >= lower, finite <= upper).all(axis=1)]
    cleaned, support_stats = filter_xyz(bounded, min_range=0, max_range=0,
                                        radius=radius, min_neighbors=min_neighbors)
    return cleaned, {
        "nonfinite": first_stats["nonfinite"],
        "outside_bounds": len(finite) - len(bounded),
        "isolated": support_stats["isolated"],
    }


def write_pcd_exclusive(path: Path, points: np.ndarray) -> None:
    """Create one new PCD, never overwriting an existing path."""
    count = len(points)
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n")
    output = path.open("xb")
    try:
        with output:
            output.write(header.encode("ascii"))
            output.write(np.asarray(points, dtype="<f4").tobytes(order="C"))
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Mapper XYZ float32 binary PCD")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--inspect", action="store_true", help="Print source counts and bounds without writing a file")
    action.add_argument("--output", type=Path, help="New cleaned PCD path; source and existing files are protected")
    parser.add_argument("--radius", type=float, default=0.2, help="Neighbor radius in metres; 0 disables isolation filtering")
    parser.add_argument("--min-neighbors", type=int, default=2, help="Required OTHER distinct points within radius")
    parser.add_argument("--bounds", type=float, nargs=6, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="Optional explicit world-coordinate crop, inclusive; no crop by default")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        source_path = args.input.expanduser().resolve()
        output_path = args.output.expanduser().resolve() if args.output is not None else None
        if output_path is not None:
            if output_path == source_path:
                raise ValueError("Output must differ from the source PCD")
            if os.path.lexists(args.output.expanduser()) or os.path.lexists(output_path):
                raise ValueError(f"Output already exists: {output_path}")
        points = read_pcd(source_path)
        report = {"source": str(source_path), "source_summary": point_summary(points)}
        if not args.inspect:
            cleaned, removed = clean_points(points, args.radius, args.min_neighbors, args.bounds)
            write_pcd_exclusive(output_path, cleaned)
            report.update({"output": str(output_path), "output_summary": point_summary(cleaned),
                           "removed": removed, "radius": args.radius, "min_neighbors": args.min_neighbors,
                           "bounds": args.bounds, "world_origin_range_filter": False})
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
