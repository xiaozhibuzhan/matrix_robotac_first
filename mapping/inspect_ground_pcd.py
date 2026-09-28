#!/usr/bin/env python3
"""Read a mapper PCD and report evidence for observed ground coverage.

Height-band counts include wall bases. Local PCA uses the full XYZ cloud to
distinguish a supported near-horizontal patch from a low vertical surface.
Neither this heuristic nor XY grid occupancy establishes complete ground truth.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial import cKDTree

try:
    from .clean_pcd import read_pcd
except ImportError:
    from clean_pcd import read_pcd


def occupied_xy_cells(points: np.ndarray, cell_size: float) -> int:
    return len(np.unique(np.floor(points[:, :2] / cell_size).astype(np.int64), axis=0))


def analyze(points: np.ndarray, ground_z: float = 0.0, tolerance: float = 0.05,
            cell_size: float = 0.2, neighbors: int = 20, radius: float = 0.3) -> tuple[dict, np.ndarray, np.ndarray]:
    if (not all(math.isfinite(value) for value in (ground_z, tolerance, cell_size, radius))
            or min(tolerance, cell_size, radius) <= 0):
        raise ValueError("Heights must be finite; tolerance, cell size and radius must be positive")
    if isinstance(neighbors, (bool, np.bool_)) or not isinstance(neighbors, (int, np.integer)) or neighbors < 3:
        raise ValueError("Neighbors must be an integer >= 3")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim == 1 and points.size == 0:
        points = points.reshape(0, 3)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points must have shape (N, 3)")
    finite = points[np.isfinite(points).all(axis=1)]
    low = finite[np.abs(finite[:, 2] - ground_z) <= tolerance]
    horizontal = np.zeros(len(low), dtype=bool)
    supported = np.zeros(len(low), dtype=bool)
    if len(finite) >= neighbors and len(low):
        distances, indices = cKDTree(finite).query(low, k=neighbors)
        supported = distances[:, -1] <= radius
        for start in range(0, len(low), 4096):
            local = finite[indices[start:start + 4096]].astype(np.float64)
            local -= local.mean(axis=1, keepdims=True)
            covariance = np.einsum("nki,nkj->nij", local, local) / neighbors
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            planar = eigenvalues[:, 0] <= 0.05 * eigenvalues.sum(axis=1)
            # Reject a line of wall-base points that cannot define a surface.
            two_dimensional = eigenvalues[:, 1] >= 0.15 * eigenvalues[:, 2]
            upright_normal = np.abs(eigenvectors[:, 2, 0]) >= math.cos(math.radians(15))
            horizontal[start:start + 4096] = supported[start:start + 4096] & planar & two_dimensional & upright_normal
    edges = np.arange(-0.1, 3.401, 0.1) + ground_z
    histogram, _ = np.histogram(finite[:, 2], edges)
    quantiles = [0, 0.001, 0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99, 1]
    all_cells = occupied_xy_cells(finite, cell_size)
    low_cells = occupied_xy_cells(low, cell_size)
    horizontal_cells = occupied_xy_cells(low[horizontal], cell_size)
    result = {
        "points": len(points),
        "finite_points": len(finite),
        "bounds_xyz_m": {"min": finite.min(axis=0).tolist(), "max": finite.max(axis=0).tolist()} if len(finite) else None,
        "z_quantiles_m": {str(q): float(v) for q, v in zip(quantiles, np.quantile(finite[:, 2], quantiles))} if len(finite) else {},
        "z_histogram": {"edges_m": edges.tolist(), "counts": histogram.tolist(), "outside_edges": int(len(finite) - histogram.sum())},
        "ground_height_band": {
            "center_z_m": ground_z, "half_width_m": tolerance,
            "points": len(low), "fraction_of_finite_points": len(low) / len(finite) if len(finite) else 0.0,
            "bounds_xyz_m": {"min": low.min(axis=0).tolist(), "max": low.max(axis=0).tolist()} if len(low) else None,
            "xy_cell_size_m": cell_size, "all_cloud_xy_cells": all_cells,
            "height_band_xy_cells": low_cells, "height_band_fraction_of_all_cloud_xy_cells": low_cells / all_cells if all_cells else 0.0,
            "nominal_occupied_cell_area_m2": low_cells * cell_size**2,
        },
        "local_horizontal_surface_evidence": {
            "neighbors": neighbors, "max_neighbor_distance_m": radius,
            "max_normal_tilt_degrees": 15, "max_smallest_eigenvalue_fraction": 0.05,
            "min_middle_to_largest_eigenvalue_ratio": 0.15,
            "height_band_points_with_neighbor_support": int(supported.sum()),
            "height_band_points_with_horizontal_patch_support": int(horizontal.sum()),
            "horizontal_patch_xy_cells": horizontal_cells,
            "nominal_occupied_cell_area_m2": horizontal_cells * cell_size**2,
        },
        "interpretation_limits": [
            "The ground height is supplied by the operator, not inferred from the minimum z.",
            "Low points can be wall bases; local PCA is a geometric heuristic, not semantic ground truth.",
            "A full-XY occupancy denominator includes walls and other structures, not a surveyed field polygon.",
            "Nominal occupied-cell area is cell count times cell area, not continuous observed surface area.",
            "Missing points cannot be reconstructed from this PCD without new sensor observations or explicit modelling.",
        ],
    }
    return result, low, horizontal


def plot_report(points: np.ndarray, low: np.ndarray, horizontal: np.ndarray, report: dict, output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    points = points[np.isfinite(points).all(axis=1)]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.3), constrained_layout=True)
    # Deterministic sampling only affects the background visualization, not statistics.
    background = points[::max(1, len(points) // 130000)]
    for ax in axes[:2]:
        ax.scatter(background[:, 0], background[:, 1], c="#bac2ca", s=0.10, rasterized=True)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("World X (m)")
        ax.set_ylabel("World Y (m)")
        ax.grid(alpha=0.2)
    axes[0].scatter(low[:, 0], low[:, 1], c="#d94801", s=2.0, label=f"Height band: {len(low):,}", rasterized=True)
    band = report["ground_height_band"]
    axes[0].set_title(f"Low points: z = {band['center_z_m']:.2f} +/- {band['half_width_m']:.2f} m")
    axes[0].legend(loc="lower left", markerscale=3)
    axes[1].scatter(low[horizontal, 0], low[horizontal, 1], c="#087f5b", s=3.0, label=f"Horizontal patch support: {horizontal.sum():,}", rasterized=True)
    axes[1].set_title("Low points supported by local horizontal geometry")
    axes[1].legend(loc="lower left", markerscale=3)
    h = report["z_histogram"]
    axes[2].stairs(h["counts"], h["edges_m"], fill=True, color="#4c6ef5")
    axes[2].axvspan(band["center_z_m"] - band["half_width_m"], band["center_z_m"] + band["half_width_m"], color="#d94801", alpha=0.3)
    axes[2].set_xlabel("World Z (m)")
    axes[2].set_ylabel("Point count per 0.1 m bin")
    axes[2].set_title("Height distribution")
    axes[2].grid(alpha=0.2)
    fig.suptitle("PCD ground evidence — measured XYZ only; low wall bases are not automatically ground", fontsize=13)
    try:
        with output.open("xb") as destination:
            fig.savefig(destination, format="png", dpi=160)
    finally:
        plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--ground-z", type=float, default=0.0, help="Expected ground height in the PCD world frame (m)")
    parser.add_argument("--tolerance", type=float, default=0.05, help="Height-band half width (m)")
    parser.add_argument("--cell-size", type=float, default=0.2, help="XY occupancy grid size (m)")
    parser.add_argument("--neighbors", type=int, default=20, help="Full-cloud nearest neighbors for local PCA")
    parser.add_argument("--radius", type=float, default=0.3, help="Maximum distance to farthest PCA neighbor (m)")
    parser.add_argument("--output-dir", type=Path,
                        help="New, nonexistent directory for ground_report.json and ground_evidence.png; requires matplotlib")
    args = parser.parse_args(argv)
    try:
        source = args.input.expanduser().resolve()
        output_dir = args.output_dir.expanduser().resolve() if args.output_dir is not None else None
        # Reject both ordinary existing paths and dangling symlinks before
        # reading the cloud; mkdir(exist_ok=False) also closes creation races.
        if output_dir is not None and (
                os.path.lexists(args.output_dir.expanduser()) or os.path.lexists(output_dir)):
            raise ValueError("--output-dir must be a new, nonexistent directory; existing files are protected")
        points = read_pcd(source)
        report, low, horizontal = analyze(points, args.ground_z, args.tolerance, args.cell_size, args.neighbors, args.radius)
        report["source_file"] = args.input.as_posix()
        report["source_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
        text = json.dumps(report, indent=2, ensure_ascii=False)
        if output_dir is not None:
            output_dir.mkdir(parents=True, exist_ok=False)
            with (output_dir / "ground_report.json").open("x", encoding="utf-8") as destination:
                destination.write(text + "\n")
            plot_report(points, low, horizontal, report, output_dir / "ground_evidence.png")
    except (OSError, ValueError, ImportError) as exc:
        parser.error(str(exc))
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
