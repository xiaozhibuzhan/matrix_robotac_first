"""Clearance preference regressions; no ROS, SDK, or gait simulation."""
import math
from pathlib import Path
import unittest
import numpy as np
from navigation.grid import GridMap


# First TRACKING poses and untouched clicked goals from 180042_hhck6zcr.
# Keep the small reproduction here; installation does not ship runtime logs.
LOG_ROUTES = (
    ((-.0500024794065763, -.16666605543761734), (2.4058258533477783, -.19393207132816315)),
    ((2.323777810641517, -.1837175904217103), (16.722003936767578, 3.6776235103607178)),
    ((16.693129689538537, 3.5986071586384822), (13.685237884521484, -5.834386348724365)),
    ((13.760620679414277, -5.826817808894509), (25.56532859802246, -8.437674522399902)),
    ((17.4202915888143, -.7842815156371317), (21.26046371459961, -.19393208622932434)),
)


def reserve(grid, path):
    return min(grid._segment_clearance(a, b) - grid.radius - grid.margin
               for a, b in zip(path, path[1:]))


class PlannerClearanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.field = GridMap.load(Path(__file__).resolve().parents[1] /
                                 'maps/task1_20260911/field_map.yaml')
        cls.original_occupancy = cls.field.occupancy.copy()
        cls.original_passable = cls.field.passable.copy()
        cls.routes = [cls.field.plan(a, b) for a, b in LOG_ROUTES]

    def test_logged_routes_remain_collision_free_with_exact_endpoints(self):
        for (start, goal), path in zip(LOG_ROUTES, self.routes):
            with self.subTest(goal=goal):
                self.assertEqual(path[0], start)
                self.assertEqual(path[-1], goal)
                self.assertTrue(all(self.field.segment_free(a, b)
                                    for a, b in zip(path, path[1:])))
                self.assertGreater(reserve(self.field, path), .05)
        np.testing.assert_array_equal(self.field.occupancy, self.original_occupancy)
        np.testing.assert_array_equal(self.field.passable, self.original_passable)

    def test_failed_fourth_goal_has_turn_reserve_and_fewer_stops(self):
        path = self.routes[3]
        # Previously: 17 points, 0.000988 m reserve. Preserve approximately
        # ten centimetres, allowing grid discretization and soft-cost tradeoffs.
        self.assertGreater(reserve(self.field, path), .085)
        self.assertLess(len(path), 17)
        self.assertLess(sum(math.dist(a, b) for a, b in zip(path, path[1:])), 23.5)
        # Recorded turn drift was only a few centimetres. It must not make the
        # outgoing interior segment cross the hard footprint boundary again.
        for a, b in zip(path[1:-1], path[2:]):
            for dx, dy in ((.025, 0.), (-.025, 0.), (0., .025), (0., -.025)):
                self.assertTrue(self.field.segment_free((a[0]+dx, a[1]+dy), b))

    def test_direct_visible_route_still_prefers_room_around_obstacle(self):
        cells = np.zeros((120, 120), dtype=np.int8)
        cells[60:80, 55:65] = 100
        grid = GridMap(cells, .05, (0., 0., 0.))
        start, goal = (1., 2.525), (5., 2.525)
        self.assertTrue(grid.segment_free(start, goal))
        self.assertLess(reserve(grid, [start, goal]), .03)
        path = grid.plan(start, goal)
        self.assertGreater(reserve(grid, path), .085)
        self.assertEqual(path[-1], goal)

    def test_narrow_legal_corridor_is_not_removed_by_preference(self):
        cells = np.full((100, 31), 100, dtype=np.int8)
        cells[5:95, 5:26] = 0
        grid = GridMap(cells, .05, (0., 0., 0.))
        before = grid.passable.copy()
        start, goal = grid.world((15, 20)), grid.world((15, 80))
        path = grid.plan(start, goal)
        self.assertGreaterEqual(reserve(grid, path), 0.)
        self.assertLess(reserve(grid, path), .1)
        self.assertEqual(path[-1], goal)
        np.testing.assert_array_equal(grid.passable, before)

    def test_low_clearance_endpoint_is_not_relocated(self):
        grid = GridMap(np.zeros((80, 100)), .05, (0., 0., 0.))
        goal = grid.world((9, 40))
        path = grid.plan((3., 2.), goal)
        self.assertEqual(path[-1], goal)
        self.assertTrue(all(grid.segment_free(a, b) for a, b in zip(path, path[1:])))

    def test_long_visible_route_has_no_artificial_four_metre_stops(self):
        path = self.field.plan((2., -.15), (12., -.15))
        self.assertEqual(len(path), 2)

    def test_collision_supercover_retains_diagonal_side_cells(self):
        cells = np.zeros((5, 5))
        cells[1, 2] = 100
        grid = GridMap(cells, 1., (0., 0., 0.), 0., 0.)
        grid.passable = cells == 0
        self.assertFalse(grid.segment_free((1.5, 1.5), (2.5, 2.5)))
        self.assertFalse(grid.segment_free((2.5, 2.5), (1.5, 1.5)))
        self.assertEqual(grid._segment_clearance((1.5, 1.5), (2.5, 2.5)), -math.inf)

    def test_grid_line_endpoints_and_nonfinite_segments_remain_safe(self):
        grid = GridMap(np.zeros((160, 160)), .05, (-4., -4., 0.))
        start, goal = (.00047, .00016), (1., 0.)
        self.assertTrue(grid.segment_free(start, goal))
        self.assertTrue(grid.segment_free(goal, start))
        self.assertFalse(grid.segment_free(start, (math.nan, 0.)))
        self.assertFalse(grid.segment_free(start, (100., 0.)))

    def test_unknown_wall_still_blocks_and_search_remains_bounded(self):
        cells = np.zeros((80, 80))
        cells[:, 40] = -1
        grid = GridMap(cells, .05, (-2., -2., 0.))
        with self.assertRaisesRegex(ValueError, 'No path'):
            grid.plan((-1., 0.), (1., 0.))
        with self.assertRaisesRegex(ValueError, 'budget'):
            grid.plan((-1., 0.), (1., 0.), max_expansions=2)


if __name__ == '__main__':
    unittest.main()
