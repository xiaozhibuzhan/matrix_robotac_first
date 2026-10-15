#!/usr/bin/env python3
"""Offline placement and reachability verification; requires navigation deps."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

# Permit both `python -m scene_tools.verify_layout` and direct script execution.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from navigation.grid import GridMap
from scene_tools.treasure_layout import read_layout, DEFAULT_LAYOUT, inspect, package_exports


def verify(project):
    project = Path(project).resolve()
    layout = read_layout(DEFAULT_LAYOUT)
    stem = project / layout["relative_stem"]
    umap, uexp = stem.with_suffix(".umap").read_bytes(), stem.with_suffix(".uexp").read_bytes()
    state = inspect(umap, uexp, layout)
    exports = package_exports(umap, layout)
    map_path = project / layout["map_yaml"]
    # Git can normalize YAML line endings; compare canonical LF text.
    yaml_hash = hashlib.sha256(map_path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    if yaml_hash != layout["map_yaml_sha256"]:
        raise ValueError("Unexpected navigation map YAML")
    if hashlib.sha256(map_path.with_suffix(".pgm").read_bytes()).hexdigest() != layout["map_pgm_sha256"]:
        raise ValueError("Unexpected navigation PGM")
    grid = GridMap.load(map_path)
    floors = {}
    for component in exports:
        if component["name"] != "DefaultSceneRoot" or component["size"] != 121:
            continue
        owner = exports[component["outer"] - 1]
        if not owner["name"].startswith("BP_Floor_3x3_C_"):
            continue
        x, y, _ = struct.unpack_from("<ddd", uexp, component["offset"] + 7)
        floors[owner["name"]] = (x / 100, -y / 100)
    if len(floors) != 37:
        raise ValueError("Floor export count mismatch")
    rows = []
    for box in layout["boxes"]:
        center, approach = box["map_center_m"], box["suggested_approach_m"]
        floor = floors[box["floor_actor"]]
        if not math.dist(floor, box["floor_center_m"]) < 1e-8:
            raise ValueError("Unexpected floor transform")
        for point, radius in ((center, .30), (approach, grid.radius + grid.margin)):
            # Adjacent 3m tiles share a floor: allow a footprint to span seams.
            for dx in np.linspace(-radius, radius, 9):
                for dy in np.linspace(-radius, radius, 9):
                    if not any(abs(point[0] + dx - f[0]) <= 1.5 + 1e-6
                               and abs(point[1] + dy - f[1]) <= 1.5 + 1e-6
                               for f in floors.values()):
                        raise ValueError("Placement/approach lacks floor support")
            if not grid.is_free(grid.cell(point)):
                raise ValueError("Placement/approach is not navigable")
        if not .75 <= math.dist(center, approach) <= 1.0 or not grid.segment_free(center, approach):
            raise ValueError("Invalid approach geometry")
        path = grid.plan((0.0, 0.0), approach)
        if not all(grid.segment_free(a, b) for a, b in zip(path, path[1:])):
            raise ValueError("Invalid route segment")
        rows.append({"id": box["id"], "center_m": center, "approach_m": approach,
                     "wall_clearance_m": float(grid.clearance[grid.cell(center)[1], grid.cell(center)[0]]),
                     "route_from_start_m": sum(math.dist(a, b) for a, b in zip(path, path[1:])),
                     "floor_actor": box["floor_actor"]})
    # Model all other, uncollected boxes as 0.30m obstacles, independently of
    # whether UE overlap-only meshes produce blocking live lidar points.
    yy, xx = np.indices(grid.occupancy.shape)
    wx = grid.origin[0] + (xx + .5) * grid.resolution
    wy = grid.origin[1] + (yy + .5) * grid.resolution
    for target, row in zip(layout["boxes"], rows):
        occupancy = grid.occupancy.copy()
        for box in layout["boxes"]:
            cx, cy = box["map_center_m"]
            # Include cell-square extent around each possible mesh footprint.
            occupancy[(wx - cx) ** 2 + (wy - cy) ** 2 <= (.30 + grid.resolution * math.sqrt(2)) ** 2] = 100
        with_boxes = GridMap(occupancy, grid.resolution, grid.origin, grid.radius, grid.margin)
        path = with_boxes.plan((0., 0.), target["suggested_approach_m"])
        row["route_with_all_boxes_m"] = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    return {"resource_state": state["state"], "floor_count": len(floors),
            "map_footprint_radius_m": grid.radius, "map_margin_m": grid.margin,
            "hypothetical_box_radius_m": .30, "boxes": rows,
            "runtime_verified": False,
            "note": "Offline check only; UE loading, rendering and overlap require target Ubuntu acceptance"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = verify(args.project)
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
