"""Package-owned image textures and bounded scene appearance parameters.

Paths in portable JSON configs are relative to task3_sim, independent of the
working directory or the config copy location. Native texture loading uses
the same byte-backed virtual filesystem as the unmodified official meshes.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np


PACKAGE = Path(__file__).resolve().parent


def asset_path(value: str) -> Path:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError("appearance assets require a package-relative POSIX path")
    path = Path(value)
    if path.is_absolute() or ":" in value or ".." in path.parts:
        raise ValueError("appearance asset must stay within task3_sim")
    resolved = (PACKAGE / path).resolve()
    if not resolved.is_relative_to(PACKAGE) or not resolved.is_file():
        raise ValueError(f"appearance asset is missing or outside task3_sim: {value}")
    return resolved


def read_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"could not decode appearance image: {path.name}")
    return image


def _scalar(value, label, minimum, maximum):
    if not isinstance(value, (float, int)) or not np.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be finite and in [{minimum}, {maximum}]")


def validate_appearance(config: dict) -> None:
    appearance = config.get("appearance", {})
    if not isinstance(appearance, dict):
        raise ValueError("appearance must be an object")
    target = appearance.get("target", {})
    if target.get("kind", "synthetic") not in ("synthetic", "image"):
        raise ValueError("appearance.target.kind must be synthetic or image")
    if target.get("kind") == "image":
        asset_path(target["asset"])
        if "calibration" in target:
            asset_path(target["calibration"])
    _scalar(target.get("brightness", 1.0), "target brightness", 0.05, 3.0)
    background = appearance.get("background")
    if background is not None:
        asset_path(background["asset"])
        for name in ("width_m", "height_m", "distance_behind_target_m"):
            _scalar(background[name], f"background {name}", 0.05, 100.0)
        _scalar(background.get("brightness", 1.0), "background brightness", 0.05, 3.0)
        tile = np.asarray(background.get("tile", [1, 1]), dtype=float)
        if tile.shape != (2,) or not np.all(np.isfinite(tile)) or np.any(tile < 0.1) or np.any(tile > 32):
            raise ValueError("background tile requires two finite values in [0.1,32]")
    lighting = appearance.get("lighting", {})
    for key in ("ambient", "diffuse"):
        _scalar(lighting.get(key, 0.5), f"lighting {key}", 0.0, 1.0)


def write_image_texture(spec: dict, output: Path) -> dict:
    """Write a derived texture; never change the supplied image itself."""
    source = asset_path(spec["asset"])
    image = read_image(source)
    gain = float(spec.get("brightness", 1.0))
    if gain != 1:
        image = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)
    output.write_bytes(cv2.imencode(".png", image)[1].tobytes())
    return {"asset": spec["asset"], "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "rendered_texture_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            "brightness": gain, "texture_size_px": [image.shape[1], image.shape[0]]}


def photo_target_texture(config: dict, output: Path) -> dict | None:
    target = config.get("appearance", {}).get("target", {})
    if target.get("kind", "synthetic") != "image":
        return None
    report = write_image_texture(target, output)
    width, height = report["texture_size_px"]
    sim = config["simulation"]
    if not np.isclose(width / height, sim["board_width_m"] / sim["board_height_m"], rtol=1e-6):
        raise ValueError("image target board dimensions must preserve the calibrated bitmap aspect")
    if "calibration" in target:
        calibration = asset_path(target["calibration"])
        report["calibration"] = target["calibration"]
        report["calibration_sha256"] = hashlib.sha256(calibration.read_bytes()).hexdigest()
    return report
