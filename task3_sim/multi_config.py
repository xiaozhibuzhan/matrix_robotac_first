"""Independent fixed-target configuration and truth-free mission contract."""
from __future__ import annotations

import copy
import json
import math

import numpy as np

from .scene import validate_config
from .appearance import asset_path


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def expand_target(config: dict, spec: dict) -> dict:
    """Resolve a board into the existing single-board geometry schema."""
    _mapping(config, "config")
    _mapping(spec, "board specification")
    result = {key: copy.deepcopy(value) for key, value in config.items()
              if key not in ("targets", "distractors", "mission")}
    for section in ("tag", "simulation", "appearance"):
        base = _mapping(result.get(section, {}), f"config.{section}")
        override = _mapping(spec.get(section, {}), f"board.{section}")
        result[section] = _merge(base, override)
    if "id" not in spec:
        raise ValueError("board specification requires an id")
    result["tag"]["id"] = spec["id"]
    # A circular synthetic board must not inherit the photograph's conics or
    # green-panel clipping polygon merely because it shares the base config.
    if result.get("appearance", {}).get("target", {}).get("kind") == "synthetic":
        for key in ("ring_ellipses_m", "scoring_polygon_m"):
            if key not in spec.get("simulation", {}):
                result["simulation"].pop(key, None)
    return result


def _id(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 587:
        raise ValueError("every board ID must be an integer in tag36h11 range 0..586")
    return value


def _finite_numbers(value, path="config"):
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_numbers(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _finite_numbers(item, f"{path}[{index}]")
    elif isinstance(value, (float, int)) and not isinstance(value, bool) and not math.isfinite(value):
        raise ValueError(f"{path} must be finite")


def validate_multi_config(config: dict) -> None:
    _mapping(config, "config")
    for key in ("camera", "laser", "tag", "control", "simulation"):
        _mapping(config.get(key), f"config.{key}")
    _mapping(config.get("appearance", {}), "config.appearance")
    background = config.get("appearance", {}).get("background")
    if isinstance(background, dict) and background.get("layout", "single_wall") not in ("single_wall", "surround"):
        raise ValueError("appearance.background.layout must be single_wall or surround")
    _finite_numbers(config)
    targets = config.get("targets")
    distractors = config.get("distractors", [])
    if not isinstance(targets, list) or not 1 <= len(targets) <= 32:
        raise ValueError("targets needs 1..32 fixed target specifications")
    if not isinstance(distractors, list) or len(distractors) > 32:
        raise ValueError("distractors must be a list with at most 32 boards")
    identities = []
    for spec in targets + distractors:
        if not isinstance(spec, dict):
            raise ValueError("board specification must be an object")
        if not isinstance(spec.get("tag_only", False), bool):
            raise ValueError("tag_only must be a boolean")
        if spec.get("tag_only", False) and spec in targets:
            raise ValueError("tag_only is reserved for non-scoring distractors")
        for section in ("tag", "simulation", "appearance"):
            _mapping(spec.get(section, {}), f"board.{section}")
        identity = _id(spec.get("id"))
        if "id" in spec.get("tag", {}) and spec["tag"]["id"] != identity:
            raise ValueError("board id and optional tag.id disagree")
        identities.append(identity)
        resolved = expand_target(config, spec)
        try:
            _mapping(resolved.get("appearance", {}).get("target", {}), "appearance.target")
            validate_config(resolved)
        except (KeyError, TypeError, AttributeError, IndexError) as error:
            raise ValueError(f"invalid configuration for board {identity}: {error}") from error
        sim, tag = resolved["simulation"], resolved["tag"]
        for name in ("board_width_m", "board_height_m"):
            value = sim[name]
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"simulation.{name} must be finite and positive")
        for name in ("board_center_down_m", "target_yaw_rad"):
            if not np.isfinite(sim[name]):
                raise ValueError(f"simulation.{name} must be finite")
        if not np.isfinite(tag["size_m"]) or tag["size_m"] <= 0:
            raise ValueError("tag.size_m must be finite and positive")
        offset = np.asarray(tag["bullseye_offset_tag_m"], dtype=float)
        # A small white quiet zone is required around every marker.
        margin = tag["size_m"] / 8
        if (abs(offset[0]) + tag["size_m"]/2 + margin >= sim["board_width_m"]/2
                or abs(-offset[1]-sim["board_center_down_m"]) + tag["size_m"]/2 + margin >= sim["board_height_m"]/2):
            raise ValueError("marker and quiet zone must fit its own board")
        image = resolved.get("appearance", {}).get("target", {})
        if image.get("kind") == "image":
            if "calibration" not in image:
                raise ValueError("image boards require marker/board metric calibration")
            calibration = json.loads(asset_path(image["calibration"]).read_text(encoding="utf-8"))
            _mapping(calibration, "image calibration")
            dimensions = _mapping(calibration.get("rectification"), "image calibration.rectification")
            for name in ("board_width_m", "board_height_m", "board_center_down_m"):
                expected = dimensions.get(name)
                if not isinstance(expected, (float, int)) or not math.isfinite(expected) or not np.isclose(sim[name], expected, rtol=1e-9, atol=1e-10):
                    raise ValueError(f"image {name} differs from its calibration; use synthetic art or provide matching image calibration")
    if len(set(identities)) != len(identities):
        raise ValueError("mission and distractor IDs must be unique across every board")
    ammo = config["simulation"]["ammo_count"]
    if isinstance(ammo, bool) or not isinstance(ammo, int) or ammo < 0:
        raise ValueError("ammo_count must be a nonnegative integer")
    mission = config.get("mission", {})
    if not isinstance(mission, dict):
        raise ValueError("mission must be an object")
    for key, value in mission.items():
        if key == "search_directions_rad":
            continue  # Signed direction array is validated by the mission contract.
        if key.endswith(("_s", "_rad_s", "_rad")) and (not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0):
            raise ValueError(f"mission.{key} must be finite and positive")
    if "max_target_attempts" in mission and (isinstance(mission["max_target_attempts"], bool) or mission["max_target_attempts"] != 1):
        raise ValueError("each fixed-target visit requires max_target_attempts=1")
    from .mission import MultiTargetMission
    MultiTargetMission(public_mission_config(config))


def public_mission_config(config: dict) -> dict:
    """Only calibration/identity/control reach the mission, never board truth."""
    result = {key: copy.deepcopy(config[key]) for key in ("camera", "laser", "tag", "control")}
    result["targets"] = [{"id": spec["id"], "tag": expand_target(config, spec)["tag"]}
                         for spec in config["targets"]]
    result["mission"] = copy.deepcopy(config.get("mission", {}))
    return result
