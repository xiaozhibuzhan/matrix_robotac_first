"""Aimed identity must never override the actual nearest physical board."""
import unittest

import mujoco
import numpy as np

from task3_sim.multi_scoring import MultiRayScorer


def referee(ammo=5, obstruction=False, overlapping=False):
    positions = {2: (3, 0, 0), 5: (3, 1, 0), 42: (3, -1, 0)}
    if overlapping:
        positions[5] = (2, 0, 0)
    bodies = []
    for identity, position in positions.items():
        bodies.append(f'<body name="task3_target_{identity}" pos="{" ".join(map(str, position))}" quat="0.707106781186548 0 0.707106781186548 0"><geom name="task3_target_collision_{identity}" type="box" size="0.3 0.3 0.001" group="2"/></body>')
    obstacle = '<geom type="box" pos="1.5 0 0" size=".05 .4 .4" group="0"/>' if obstruction else ''
    # Transparent-to-referee background and robot are deliberately in front.
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
        <body name="base_link"><geom type="sphere" size=".1" group="3"/></body>
        <geom type="box" pos="1 0 0" size=".01 .4 .4" group="1"/>
        {obstacle}{''.join(bodies)}</worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    config = {"laser": {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]},
              "simulation": {"ammo_count": ammo, "ring_radii_m": [.02, .04, .06, .08, .1]},
              "tag": {"id": 2}, "targets": [{"id": 2}, {"id": 5}], "distractors": [{"id": 42}]}
    return MultiRayScorer(model, data, config), config


class MultiRefereeTests(unittest.TestCase):
    def test_wrong_identity_keeps_physical_ring_and_credits_actual_only(self):
        scorer, _ = referee()
        scorer.shooting_init()
        result = scorer.shooting_fire(5)
        self.assertEqual((result.target_id, result.ring, result.reason), (2, 10, "wrong_target"))
        self.assertEqual(scorer.scores, {2: 10})
        self.assertEqual(scorer.ammo_remaining, 4)
        self.assertEqual(scorer.ledger[0]["intended_target_id"], 5)
        self.assertEqual(scorer.ledger[0]["actual_target_id"], 2)

    def test_two_boards_in_line_nearest_target_occludes_intended_board(self):
        scorer, _ = referee(overlapping=True)
        scorer.shooting_init()
        result = scorer.shooting_fire(2)
        self.assertEqual((result.target_id, result.ring, result.reason), (5, 10, "wrong_target"))
        np.testing.assert_allclose(result.point_world_m, [1.999, 0, 0], atol=1e-9)
        self.assertNotIn(2, scorer.scores)

    def test_unknown_distractor_never_earns_mission_credit(self):
        scorer, config = referee()
        config["laser"]["direction_base"] = [1, -1/2.999, 0]
        scorer.shooting_init()
        result = scorer.shooting_fire(2)
        self.assertEqual((result.target_id, result.ring, result.hit, result.reason), (42, 10, False, "distractor"))
        self.assertEqual(scorer.scores, {})
        self.assertEqual(scorer.ledger[0]["physical_ring"], 10)

    def test_world_obstacle_spends_one_round_without_fabricated_target_identity(self):
        scorer, _ = referee(obstruction=True)
        scorer.shooting_init()
        result = scorer.shooting_fire(2)
        self.assertEqual((result.target_id, result.ring, result.reason), (None, 3, "occluded"))
        self.assertEqual(scorer.scores, {})
        self.assertEqual(scorer.ledger[0]["ammo_before"], 5)
        self.assertEqual(scorer.ledger[0]["ammo_after"], 4)

    def test_empty_uninitialized_and_unknown_intention_never_spend_ammo(self):
        scorer, _ = referee(ammo=1)
        self.assertEqual(scorer.shooting_fire(2).reason, "not_initialized")
        self.assertEqual(scorer.shooting_fire(42).reason, "unknown_intended_target")
        self.assertEqual(scorer.ammo_remaining, 1)
        scorer.shooting_init()
        scorer.shooting_fire(2)
        self.assertEqual(scorer.shooting_fire(5).reason, "no_ammo")
        self.assertEqual(scorer.shooting_reload(), -1)
        self.assertEqual((scorer.ammo_remaining, len(scorer.shots), len(scorer.ledger)), (0, 1, 1))

    def test_off_board_ray_records_ring3_and_every_ledger_round_balances(self):
        scorer, config = referee(ammo=2)
        scorer.shooting_init()
        scorer.shooting_fire(2)
        config["laser"]["direction_base"] = [0, 1, 0]
        result = scorer.shooting_fire(5)
        self.assertEqual((result.target_id, result.ring, result.reason), (None, 3, "no_surface"))
        self.assertEqual(scorer.scores, {2: 10})
        self.assertEqual([r["sequence"] for r in scorer.ledger], [1, 2])
        self.assertTrue(all(r["ammo_before"]-r["ammo_after"] == 1 for r in scorer.ledger))


if __name__ == "__main__":
    unittest.main()
