"""Deterministic, read-only import of oversized official binary STL assets.

MuJoCo accepts at most 200,000 STL faces. Only an oversized mesh is derived:
vertices are rounded to a metric grid, collapsed triangles and duplicate
faces are removed, and normals are recalculated. Original bytes are never
written. The caller can persist the returned derivative beside its run XML.
"""

from __future__ import annotations

import hashlib
import struct

import numpy as np


_FACE_DTYPE = np.dtype([
    ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2"),
])


def prepare_stl(raw: bytes, max_faces: int = 200000) -> tuple[bytes, dict]:
    """Return a loadable binary STL plus an auditable approximation report.

    Coordinates of these official xgb STL assets are in metres. A 1 mm grid
    is attempted first and enlarged only when needed to satisfy the face
    limit. The maximum displacement is measured, not just an upper bound.
    Meshes already below the limit are returned byte-for-byte unchanged.
    """
    if max_faces < 4:
        raise ValueError("max_faces must be at least four")
    if len(raw) < 84:
        raise ValueError("binary STL is shorter than its header")
    original_faces = struct.unpack_from("<I", raw, 80)[0]
    if original_faces < 1 or len(raw) != 84 + 50 * original_faces:
        raise ValueError("expected a binary STL with a consistent face count")
    source_hash = hashlib.sha256(raw).hexdigest()
    report = {
        "source_sha256": source_hash,
        "original_faces": original_faces,
        "derived_faces": original_faces,
        "grid_spacing_m": 0.0,
        "max_vertex_displacement_m": 0.0,
        "changed": False,
        "method": "unchanged",
    }
    if original_faces <= max_faces:
        return raw, report
    records = np.frombuffer(raw, dtype=_FACE_DTYPE, count=original_faces, offset=84)
    original_vertices = np.asarray(records["vertices"], dtype=np.float64).reshape(-1, 3)
    if not np.all(np.isfinite(original_vertices)):
        raise ValueError("STL contains non-finite vertices")
    grid = 0.001
    for _ in range(32):
        quantized = np.rint(original_vertices / grid).astype(np.int64)
        grid_vertices, indices = np.unique(quantized, axis=0, return_inverse=True)
        triangle_indices = indices.reshape(-1, 3)
        nondegenerate = (triangle_indices[:, 0] != triangle_indices[:, 1]) & (
            triangle_indices[:, 1] != triangle_indices[:, 2]) & (
            triangle_indices[:, 2] != triangle_indices[:, 0])
        triangle_indices = triangle_indices[nondegenerate]
        # Sorting only for the duplicate key keeps the first original winding.
        _, first = np.unique(np.sort(triangle_indices, axis=1), axis=0, return_index=True)
        triangle_indices = triangle_indices[np.sort(first)]
        points = grid_vertices[triangle_indices].astype(np.float64) * grid
        normals = np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        noncollinear = lengths > 1e-15
        points, normals, lengths = points[noncollinear], normals[noncollinear], lengths[noncollinear]
        if 0 < len(points) <= max_faces:
            displacement = np.linalg.norm(quantized * grid - original_vertices, axis=1)
            result = np.zeros(len(points), dtype=_FACE_DTYPE)
            result["vertices"] = points
            result["normal"] = normals / lengths[:, None]
            header = b"task3 derived vertex-grid STL; official source preserved".ljust(80, b" ")
            derived = header + struct.pack("<I", len(points)) + result.tobytes()
            report.update({
                "derived_faces": len(points),
                "grid_spacing_m": grid,
                "max_vertex_displacement_m": float(np.max(displacement)),
                "changed": True,
                "method": "vertex_grid_clustering_remove_degenerate_duplicate_faces",
                "derived_sha256": hashlib.sha256(derived).hexdigest(),
            })
            return derived, report
        if not len(points):
            raise ValueError("mesh collapsed before satisfying the requested face limit")
        grid *= 1.5
    raise ValueError("could not reduce mesh below the requested face limit")
