"""Compare batched display reduction with the previous scalar ownership rules."""

import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_field_mapper import mapper


def display_state():
    return SimpleNamespace(
        voxels={}, voxel_size=0.03, cell_size=0.2, display_voxel=0.06,
        surface_owners={}, surface_samples={}, cells={}, _display_owners={}, _display_points={},
        _surface_buckets_by_owner={}, _display_buckets_by_owner={}, _registered_points=0,
        fusion=SimpleNamespace(last_changed_keys=(), last_added_keys=(), last_changed_points=np.empty((0, 3))),
    )


def scalar_update(state, changed):
    # Fixed fine-voxel centres define ownership, independent of the point mean.
    for key in changed:
        point = state.voxels[key]
        centre = tuple((coordinate + 0.5) * state.voxel_size for coordinate in key)
        surface_key = tuple(math.floor(coordinate / state.cell_size) for coordinate in centre)
        owner = state.surface_owners.setdefault(surface_key, key)
        if owner == key:
            state.surface_samples[surface_key] = point
        cell = state.cells.setdefault(surface_key[:2], {"min_z": point[2], "max_z": point[2]})
        cell["min_z"] = min(float(cell["min_z"]), point[2])
        cell["max_z"] = max(float(cell["max_z"]), point[2])
        display_key = tuple(math.floor(coordinate / state.display_voxel) for coordinate in centre)
        owner = state._display_owners.setdefault(display_key, key)
        if owner == key:
            state._display_points[display_key] = point


class DisplayBatchTests(unittest.TestCase):
    def setUp(self):
        self.batch = display_state()
        self.scalar = display_state()

    def update(self, changes, cached=False):
        keys = list(changes)
        self.batch.fusion.last_added_keys = tuple(key for key in keys if key not in self.batch.voxels)
        self.batch.voxels.update(changes)
        self.scalar.voxels.update(changes)
        if cached:
            self.batch.fusion.last_changed_keys = tuple(keys)
            self.batch.fusion.last_changed_points = np.asarray(list(changes.values()), dtype=np.float64)
        else:
            self.batch.fusion.last_changed_keys = ()
        mapper.FieldMapperNode.update_display(self.batch, keys)
        scalar_update(self.scalar, keys)
        for name in ("surface_owners", "surface_samples", "cells", "_display_owners", "_display_points"):
            self.assertEqual(getattr(self.batch, name), getattr(self.scalar, name), name)

    def test_empty_update_changes_nothing(self):
        before = copy.deepcopy(self.batch.cells)
        self.update({})
        self.assertEqual(self.batch.cells, before)

    def test_negative_coordinates_and_six_and_twenty_centimetre_boundaries(self):
        changes = {}
        for x in (-8, -7, -6, -3, -2, -1, 0, 1, 2, 5, 6, 7):
            for y in (-7, -2, -1, 0, 1, 2, 6, 7):
                key = (x, y, -1 if x % 2 else 0)
                changes[key] = tuple((coordinate + 0.1) * 0.03 for coordinate in key)
        self.update(changes, cached=True)
        self.assertTrue(any(any(axis < 0 for axis in key) for key in self.batch._display_points))

    def test_existing_owner_absent_from_changes_keeps_measured_representative(self):
        owner = (0, 0, 0)
        self.update({owner: (0.004, 0.008, 0.002)})
        self.update({(1, 1, 1): (0.058, 0.059, 0.052)}, cached=True)
        self.assertEqual(self.batch._display_points[(0, 0, 0)], (0.004, 0.008, 0.002))
        self.assertEqual(self.batch.cells[(0, 0)], {"min_z": 0.002, "max_z": 0.052})

    def test_mean_crossing_coarse_boundary_does_not_duplicate_owner_or_erase_history(self):
        key = (6, 0, 6)  # 18--21 cm fine voxel straddles a 20 cm cell edge.
        self.update({key: (0.199, 0.015, 0.199)})
        self.update({key: (0.201, 0.015, 0.201)}, cached=True)
        self.update({key: (0.195, 0.015, 0.195)})
        self.assertEqual(len(self.batch.surface_samples), 1)
        self.assertEqual(len(self.batch._display_points), 1)
        self.assertEqual(self.batch.cells[(0, 0)], {"min_z": 0.195, "max_z": 0.201})

    def test_three_random_frames_preserve_first_owner_and_all_height_extrema(self):
        random = np.random.default_rng(9731)
        keys = list(dict.fromkeys(map(tuple, random.integers(-25, 26, size=(2500, 3)).tolist())))
        for frame in range(3):
            random.shuffle(keys)
            selected = keys[:1800]
            changes = {
                key: tuple(((np.asarray(key) + random.uniform(0.05, 0.95, 3)) * 0.03).tolist())
                for key in selected
            }
            self.update(changes, cached=bool(frame % 2))


if __name__ == "__main__":
    unittest.main()
