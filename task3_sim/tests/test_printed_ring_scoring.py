"""Image artwork can have elliptical rings; its ray scores must follow them."""
import copy
import unittest
import numpy as np

from task3_sim.scoring import ring_for_point, validate_ring_ellipses


def artwork_simulation():
    return {"board_width_m": 1.0, "board_height_m": 1.0, "board_center_down_m": 0.0,
            "ring_radii_m": [0.04, 0.08, 0.12, 0.16, 0.20],
            "ring_ellipses_m": [{"center_offset_m": [0.01, -0.02],
                                 "quadratic_m_inv2": np.diag([1/(0.04*i)**2, 1/(0.03*i)**2]).tolist()}
                                for i in range(1, 6)]}


class PrintedRingTests(unittest.TestCase):
    def test_truncated_ring_artwork_does_not_score_on_frame_or_tag(self):
        simulation = artwork_simulation()
        simulation["scoring_polygon_m"] = [[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]]
        validate_ring_ellipses(simulation)
        self.assertEqual(ring_for_point([0.01, -0.02], simulation), 10)
        self.assertEqual(ring_for_point([0.1, -0.02], simulation), 8)
        self.assertEqual(ring_for_point([0.15, -0.02], simulation), 3)

    def test_inner_boundary_and_ellipse_axes_follow_printed_artwork(self):
        simulation = artwork_simulation()
        validate_ring_ellipses(simulation)
        self.assertEqual(ring_for_point([0.05, -0.02], simulation), 10)
        self.assertEqual(ring_for_point([0.01, 0.01], simulation), 10)
        self.assertEqual(ring_for_point([0.01, 0.015], simulation), 9)
        self.assertEqual(ring_for_point([0.23, -0.02], simulation), 3)
        self.assertEqual(ring_for_point([0.0, 0.0], {"ring_radii_m": [0.02, .04, .06, .08, .1]}), 10)

    def test_invalid_non_nested_or_off_board_conics_are_rejected(self):
        base = artwork_simulation()
        mutations = [lambda s:s["ring_ellipses_m"].pop(),
                     lambda s:s["ring_ellipses_m"][0].update(quadratic_m_inv2=[[-1, 0], [0, 1]]),
                     lambda s:s["ring_ellipses_m"][0].update(quadratic_m_inv2=[[1, 2], [0, 1]]),
                     lambda s:s["ring_ellipses_m"][0].update(center_offset_m=[0.4, 0.4]),
                     lambda s:s["ring_ellipses_m"][-1].update(center_offset_m=[0.9, 0.0])]
        for mutation in mutations:
            simulation = copy.deepcopy(base)
            mutation(simulation)
            with self.subTest(simulation=simulation), self.assertRaises(ValueError):
                validate_ring_ellipses(simulation)


if __name__ == "__main__":
    unittest.main()
