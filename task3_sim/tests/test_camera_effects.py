"""Sensor stress must preserve geometry and have reproducible, changing noise."""
import unittest
import numpy as np

from task3_sim.sensor import CameraEffects


class CameraEffectTests(unittest.TestCase):
    def test_identity_is_lossless_and_other_effects_do_not_mutate_render(self):
        rgb = np.random.default_rng(4).integers(0, 256, (48, 64, 3), dtype=np.uint8)
        original = rgb.copy()
        np.testing.assert_array_equal(CameraEffects().apply(rgb), rgb)
        altered = CameraEffects({"exposure": 0.7, "gamma": 1.1, "blur_sigma_px": 0.6}).apply(rgb)
        np.testing.assert_array_equal(rgb, original)
        self.assertEqual(altered.shape, rgb.shape)
        self.assertEqual(altered.dtype, np.uint8)
        self.assertLess(float(altered.mean()), float(rgb.mean()))

    def test_noise_is_repeatable_but_changes_across_frames_without_uint8_wrap(self):
        rgb = np.full((32, 48, 3), 250, dtype=np.uint8)
        first = CameraEffects({"noise_std_u8": 12.0, "seed": 19})
        second = CameraEffects({"noise_std_u8": 12.0, "seed": 19})
        a, b = first.apply(rgb), first.apply(rgb)
        np.testing.assert_array_equal(a, second.apply(rgb))
        np.testing.assert_array_equal(b, second.apply(rgb))
        self.assertFalse(np.array_equal(a, b))
        self.assertGreater(int(a.min()), 180)
        self.assertEqual(int(a.max()), 255)

    def test_invalid_or_unbounded_settings_are_rejected(self):
        for settings in ({"exposure": float("nan")}, {"blur_sigma_px": -1},
                         {"noise_std_u8": 1000}, {"seed": True}, {"seed": -1},
                         {"gamma": 0}, {"target_position": [1, 2, 3]}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                CameraEffects(settings)


if __name__ == "__main__":
    unittest.main()
