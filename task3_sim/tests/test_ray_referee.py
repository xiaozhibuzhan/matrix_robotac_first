"""The referee must score actual nearest-surface ray hits and finite ammunition."""
import unittest

import mujoco
import numpy as np

from task3_sim.scoring import RayScorer, ring_for_radius


def referee(obstacle=False, ammo=5):
    obstruction = '<geom name="obstacle" type="box" pos="1.5 0 0" size="0.05 0.5 0.5"/>' if obstacle else ''
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body name="base_link"><geom type="sphere" size="0.1" group="3"/></body>
      {obstruction}
      <body name="task3_target" pos="3 0 0" quat="0.707106781186548 0 0.707106781186548 0">
        <geom name="task3_target_collision" type="box" size="0.4 0.4 0.001"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    config = {"laser": {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]},
              "simulation": {"ammo_count": ammo, "ring_radii_m": [.02, .04, .06, .08, .1]},
              "tag": {"id": 0}}
    scorer = RayScorer(model, data, config)
    return scorer, config


class RayRefereeTests(unittest.TestCase):
    def test_center_ring_and_boundary_policy(self):
        self.assertEqual(ring_for_radius(0, [.02, .04, .06, .08, .1]), 10)
        self.assertEqual(ring_for_radius(.04, [.02, .04, .06, .08, .1]), 9)
        self.assertEqual(ring_for_radius(.1001, [.02, .04, .06, .08, .1]), 3)
        scorer, _ = referee()
        scorer.shooting_init()
        result = scorer.shooting_fire()
        self.assertTrue(result.hit)
        self.assertEqual(result.ring, 10)
        np.testing.assert_allclose(result.point_world_m, [2.999, 0, 0], atol=1e-8)

    def test_obstacle_blocks_target_and_spends_one_round(self):
        scorer, _ = referee(obstacle=True)
        scorer.shooting_init()
        result = scorer.shooting_fire()
        self.assertFalse(result.hit)
        self.assertEqual(result.ring, 3)
        self.assertEqual(result.reason, "occluded")
        self.assertEqual(scorer.ammo_remaining, 4)
        self.assertLess(result.point_world_m[0], 2)

    def test_later_worse_ring_replaces_previous_ring(self):
        scorer, config = referee()
        scorer.shooting_init()
        self.assertEqual(scorer.shooting_fire().ring, 10)
        config["laser"]["direction_base"] = [1, 0.05/2.999, 0]
        self.assertEqual(scorer.shooting_fire().ring, 8)
        self.assertEqual(scorer.last_scores[0], 8)

    def test_spent_miss_replaces_intended_single_target_score(self):
        scorer, config = referee()
        scorer.shooting_init()
        self.assertEqual(scorer.shooting_fire().ring, 10)
        config["laser"]["direction_base"] = [0, 1, 0]
        missed = scorer.shooting_fire()
        self.assertIsNone(missed.target_id)
        self.assertEqual(missed.ring, 3)
        self.assertEqual(scorer.last_scores[0], 3)
        self.assertEqual(scorer.ammo_remaining, 3)

    def test_no_implicit_reload_or_uninitialized_shot(self):
        scorer, _ = referee(ammo=1)
        self.assertEqual(scorer.shooting_fire().reason, "not_initialized")
        self.assertEqual(scorer.ammo_remaining, 1)
        scorer.shooting_init()
        scorer.shooting_fire()
        self.assertEqual(scorer.shooting_reload(), -1)
        self.assertEqual(scorer.shooting_fire().reason, "no_ammo")
        self.assertEqual(scorer.ammo_remaining, 0)
        self.assertEqual(len(scorer.shots), 1)


if __name__ == "__main__":
    unittest.main()
