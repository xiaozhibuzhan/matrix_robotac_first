"""Repeatable camera stress applied to actual MuJoCo RGB observations."""

from __future__ import annotations

import math
import numpy as np


class CameraEffects:
    """Simple photometric/blur/noise model, not a calibrated physical camera."""

    def __init__(self, settings=None):
        settings = {} if settings is None else settings
        limits = {"exposure": (0.05, 4.0, 1.0), "gamma": (0.1, 4.0, 1.0),
                  "blur_sigma_px": (0.0, 5.0, 0.0), "noise_std_u8": (0.0, 80.0, 0.0)}
        if not isinstance(settings, dict) or set(settings) - {*limits, "seed"}:
            raise ValueError("unknown camera_effects setting")
        for name, (low, high, default) in limits.items():
            value = float(settings.get(name, default))
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"camera_effects.{name} must be finite in [{low}, {high}]")
            setattr(self, name, value)
        seed = settings.get("seed", 0)
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**63:
            raise ValueError("camera_effects.seed must be an integer in [0, 2**63)")
        self.rng = np.random.default_rng(seed)

    def apply(self, rgb):
        """Keep resolution, axes and timestamps; never add target information."""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("camera effects require an RGB uint8 image")
        if self.exposure == self.gamma == 1.0 and self.blur_sigma_px == self.noise_std_u8 == 0:
            return rgb
        image = rgb.astype(np.float32)
        if self.blur_sigma_px > 0:
            import cv2
            image = cv2.GaussianBlur(image, (0, 0), self.blur_sigma_px)
        image = 255.0 * np.power(np.clip(image * (self.exposure / 255.0), 0, 1), self.gamma)
        if self.noise_std_u8 > 0:
            image += self.rng.normal(0.0, self.noise_std_u8, image.shape).astype(np.float32)
        return np.rint(np.clip(image, 0, 255)).astype(np.uint8)
