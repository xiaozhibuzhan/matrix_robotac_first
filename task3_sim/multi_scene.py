"""Render independent fixed boards using the unmodified official robot model."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

from .appearance import asset_path, photo_target_texture, read_image
from .multi_config import expand_target, validate_multi_config
from .scene import build_scene, make_target_texture, numbers, quaternion, rpy_matrix


def _target_texture(config: dict, path: Path) -> dict:
    """Keep ID 2's supplied art; change only a derived marker patch for other IDs."""
    report = photo_target_texture(config, path)
    if report is None:
        make_target_texture(config, path)
        return {"kind": "synthetic", "id": config["tag"]["id"],
                "rendered_texture_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    spec = config["appearance"]["target"]
    if "calibration" not in spec:
        raise ValueError("multi-target image artwork needs marker calibration provenance")
    calibration = json.loads(asset_path(spec["calibration"]).read_text(encoding="utf-8"))
    original = calibration["target_tag"]
    sim, tag = config["simulation"], config["tag"]
    source_offset = np.asarray(calibration["bullseye"]["offset_tag_m"], dtype=float)
    requested_offset = np.asarray(tag["bullseye_offset_tag_m"], dtype=float)
    marker_unchanged = (tag["id"] == original["id"] and np.allclose(requested_offset, source_offset, atol=1e-10)
                        and np.isclose(tag["size_m"], original["black_square_size_m"], atol=1e-10))
    report["kind"] = "image"
    report["id"] = int(tag["id"])
    report["marker_replaced"] = not marker_unchanged
    if marker_unchanged:
        report["transformation"] = "supplied rectified artwork retained, optional brightness only"
        return report
    image = read_image(path)
    height, width = image.shape[:2]
    # Board +X points right, +Y down. The same calibrated metric mapping is
    # used by the renderer, pose-to-bullseye offset, and referee.
    scale = np.array([width/sim["board_width_m"], height/sim["board_height_m"]])
    origin = np.array([width/2, height/2-sim["board_center_down_m"]*scale[1]])
    rectification = calibration["rectification"]
    original_scale = np.array([width/rectification["board_width_m"], height/rectification["board_height_m"]])
    original_origin = np.array([width/2, height/2-rectification["board_center_down_m"]*original_scale[1]])
    original_center = original_origin-source_offset[:2]*original_scale
    original_size = float(original["black_square_size_m"])*original_scale
    old_start = np.floor(original_center-original_size/2-2).astype(int)
    old_end = np.ceil(original_center+original_size/2+2).astype(int)
    if np.any(old_start < 0) or old_end[0] >= width or old_end[1] >= height:
        raise ValueError("calibrated original marker lies outside board texture")
    gain = float(spec.get("brightness", 1))
    image[old_start[1]:old_end[1]+1, old_start[0]:old_end[0]+1] = np.clip(255*gain, 0, 255)
    center = origin-requested_offset[:2]*scale
    half_size = float(tag["size_m"])*scale/2
    # Shifted markers can extend beyond the photograph's original white panel.
    # Supply one full tag cell of white quiet zone around the new black square;
    # otherwise a black edge touching the green panel is not reliably decoded.
    quiet_zone = float(tag["size_m"])*scale/8
    quiet_start = np.floor(center-half_size-quiet_zone).astype(int)
    quiet_end = np.ceil(center+half_size+quiet_zone).astype(int)
    if np.any(quiet_start < 0) or quiet_end[0] >= width or quiet_end[1] >= height:
        raise ValueError("derived marker and one-cell white quiet zone must fit the image board")
    image[quiet_start[1]:quiet_end[1]+1, quiet_start[0]:quiet_end[0]+1] = np.clip(255*gain, 0, 255)
    # The derived patch includes its declared quiet zone. All pixels outside
    # this patch and the erased original marker remain as supplied.
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), int(tag["id"]), 320)
    marker = cv2.rotate(marker, cv2.ROTATE_180)
    marker = np.clip(marker.astype(np.float32)*gain, 0, 255).astype(np.uint8)
    destination = np.array([center-half_size, center+[half_size[0], -half_size[1]],
                            center+half_size, center+[-half_size[0], half_size[1]]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(np.array([[0, 0], [319, 0], [319, 319], [0, 319]], np.float32), destination)
    warped = cv2.warpPerspective(marker, transform, (width, height), flags=cv2.INTER_NEAREST)
    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, np.rint(destination).astype(np.int32), 255)
    image[mask != 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask != 0]
    path.write_bytes(cv2.imencode(".png", image)[1].tobytes())
    report.update({"transformation": "derived marker plus one-cell white quiet-zone patch; rotated 180 degrees to match AprilTag3 metric axes; original source asset unchanged",
                   "original_marker_id": int(original["id"]), "marker_size_m": float(tag["size_m"]),
                   "marker_corners_texture_px": destination.tolist(), "erased_original_patch_px": [old_start.tolist(), old_end.tolist()],
                   "quiet_zone_cell_fraction": .125, "quiet_zone_patch_px": [quiet_start.tolist(), quiet_end.tolist()],
                   "rendered_texture_sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    return report


def _tag_only_texture(config: dict, path: Path) -> dict:
    """A white one-cell quiet zone around a marker, without target artwork."""
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), int(config["tag"]["id"]), 400)
    marker = cv2.rotate(marker, cv2.ROTATE_180)
    image = np.full((500, 500), 255, dtype=np.uint8)
    image[50:450, 50:450] = marker
    gain = float(config.get("appearance", {}).get("target", {}).get("brightness", 1.0))
    image = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)
    path.write_bytes(cv2.imencode(".png", image)[1].tobytes())
    return {"kind": "tag_only", "id": int(config["tag"]["id"]),
            "marker_size_m": float(config["tag"]["size_m"]),
            "patch_side_m": float(config["tag"]["size_m"]) * 1.25,
            "quiet_zone_cell_fraction": .125, "scoring_surface": False,
            "rendered_texture_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _append_board(world, assets, config, texture_path, *, tag_only=False):
    sim, identity = config["simulation"], int(config["tag"]["id"])
    rotation = rpy_matrix([0, 0, sim["target_yaw_rad"]]) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    body_name = f"task3_distractor_tag_{identity}" if tag_only else f"task3_target_{identity}"
    board = ET.SubElement(world, "body", name=body_name, pos=numbers(sim["target_center_world_m"]), quat=numbers(quaternion(rotation)))
    half_w, half_h, down = sim["board_width_m"]/2, sim["board_height_m"]/2, sim["board_center_down_m"]
    front = [[-half_w, down-half_h, 0], [half_w, down-half_h, 0], [half_w, down+half_h, 0], [-half_w, down+half_h, 0]]
    vertices = front + [[x, y, 0.012] for x, y, _ in front]
    faces = [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7]
    ET.SubElement(assets, "texture", name=f"task3_board_texture_{identity}", type="2d", file=str(texture_path.resolve()))
    ET.SubElement(assets, "material", name=f"task3_board_material_{identity}", texture=f"task3_board_texture_{identity}", texuniform="false", emission="1", specular="0", shininess="0")
    ET.SubElement(assets, "mesh", name=f"task3_board_mesh_{identity}", vertex=numbers(vertices), face=" ".join(map(str, faces)), texcoord=numbers([[0, 0], [1, 0], [1, 1], [0, 1]]*2))
    visual_name = f"task3_distractor_visual_{identity}" if tag_only else f"task3_board_visual_{identity}"
    ET.SubElement(board, "geom", name=visual_name, type="mesh", mesh=f"task3_board_mesh_{identity}", material=f"task3_board_material_{identity}", contype="0", conaffinity="0", group="1")
    if not tag_only:
        ET.SubElement(board, "geom", name=f"task3_target_collision_{identity}", type="box", pos=numbers([0, down, 0.006]), size=numbers([half_w, half_h, 0.006]), rgba="0.2 0.2 0.2 1", group="2")


def build_multi_scene(config: dict, model_root: Path, output: Path) -> tuple[mujoco.MjModel, dict]:
    validate_multi_config(config)
    output = Path(output)
    first = expand_target(config, config["targets"][0])
    # Reuse the established read-only official-model import and repaired mesh
    # adapter. Only package-owned generated MJCF is transformed below.
    initial_model, provenance = build_scene(first, Path(model_root), output)
    # Conservative enclosing radii cover the actual imported robot's home
    # geometry at its configured starting pose, without creating a renderer.
    initial_data = mujoco.MjData(initial_model)
    initial_data.qpos[:] = initial_model.key_qpos[0] if initial_model.nkey else initial_model.qpos0
    initial_data.qpos[:3] = config["simulation"]["initial_position_m"]
    initial_data.qpos[3:7] = quaternion(rpy_matrix(config["simulation"]["initial_rpy_rad"]))
    mujoco.mj_forward(initial_model, initial_data)
    robot_geoms = initial_model.geom_group == 3
    robot_bounds_xy = float(np.max(np.abs(initial_data.geom_xpos[robot_geoms, :2])+initial_model.geom_rbound[robot_geoms, None]))
    robot_top = float(np.max(initial_data.geom_xpos[robot_geoms, 2]+initial_model.geom_rbound[robot_geoms]))
    del initial_data
    del initial_model
    tree = ET.parse(output/"scene.xml")
    root = tree.getroot()
    root.set("model", "task3_independent_multi_fixed_targets")
    world, assets = root.find("worldbody"), root.find("asset")
    original_board = world.find("body[@name='task3_target']")
    background = original_board.find("geom[@name='task3_background_visual']")
    background_placement = None
    background_spec = first.get("appearance", {}).get("background")
    board_corners = []
    for spec in config["targets"] + config.get("distractors", []):
        simulation = expand_target(config, spec)["simulation"]
        rotation = rpy_matrix([0, 0, simulation["target_yaw_rad"]]) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        center = np.asarray(simulation["target_center_world_m"], dtype=float)
        half_width, half_height = simulation["board_width_m"]/2, simulation["board_height_m"]/2
        for x in (-half_width, half_width):
            for y in (simulation["board_center_down_m"]-half_height, simulation["board_center_down_m"]+half_height):
                for z in (0, .012):
                    board_corners.append(center+rotation @ [x, y, z])
    board_corners = np.asarray(board_corners)
    if background is not None and background_spec.get("layout") == "surround":
        # The four clockwise walls enclose every board corner and the imported
        # robot geometry. Their front faces point into the room; the thin
        # closed meshes also remain visible from the outside overview camera.
        margin = float(background_spec["distance_behind_target_m"])
        board_extent = float(np.max(np.abs(board_corners[:, :2])))
        radius = max(board_extent, robot_bounds_xy)+margin
        height = max(float(background_spec["height_m"]), float(np.max(board_corners[:, 2]))+margin, robot_top+margin)
        mesh = assets.find("mesh[@name='task3_background_mesh']")
        width = 2*radius
        front = [[-width/2, -height/2, 0], [width/2, -height/2, 0],
                 [width/2, height/2, 0], [-width/2, height/2, 0]]
        mesh.set("vertex", numbers(front+[[x, y, .02] for x, y, _ in front]))
        # All walls share the existing supplied image, material, UV mapping
        # and group-1 visual-only geometry. No image is composited onto frames.
        walls = []
        for name, position, yaw in (("front", [radius, 0, height/2], 0.),
                                     ("left", [0, radius, height/2], np.pi/2),
                                     ("back", [-radius, 0, height/2], np.pi),
                                     ("right", [0, -radius, height/2], -np.pi/2)):
            rotation = rpy_matrix([0, 0, yaw]) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
            wall = ET.SubElement(world, "body", name=f"task3_background_{name}", pos=numbers(position), quat=numbers(quaternion(rotation)))
            visual = copy.deepcopy(background)
            visual.set("name", f"task3_background_visual_{name}")
            wall.append(visual)
            walls.append({"name": name, "center_world_m": position, "outward_normal_world": rotation[:, 2].tolist()})
        background_placement = {"layout": "surround", "room_half_extent_m": radius,
                                "room_bounds_xy_m": [[-radius, -radius], [radius, radius]],
                                "wall_width_m": width, "wall_height_m": height,
                                "board_max_xy_corner_extent_m": board_extent,
                                "robot_max_xy_geometry_extent_m": robot_bounds_xy,
                                "distance_behind_boards_m": margin, "walls": walls,
                                "policy": "four textured visual-only walls beyond every board corner and imported robot geometry"}
    elif background is not None:
        # A single real textured wall is shared by all boards, not a copy for
        # every target; visual group 1 never participates in scoring rays.
        # Preserve the common lateral center, but never average board depths:
        # an intentionally invisible board behind the robot must not pull the
        # background in front of the remaining visible mission boards.
        wall_rotation = rpy_matrix([0, 0, first["simulation"]["target_yaw_rad"]]) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        wall_normal = wall_rotation[:, 2]
        wall_center = np.mean([expand_target(config, spec)["simulation"]["target_center_world_m"] for spec in config["targets"]], axis=0)
        furthest_depth = float(np.max(board_corners @ wall_normal))
        wall_center += (furthest_depth-float(wall_center @ wall_normal))*wall_normal
        wall = ET.SubElement(world, "body", name="task3_background", pos=numbers(wall_center), quat=original_board.get("quat"))
        wall.append(copy.deepcopy(background))
        background_placement = {"anchor_world_m": wall_center.tolist(), "normal_world": wall_normal.tolist(),
                                "furthest_board_surface_projection_m": furthest_depth,
                                "distance_behind_boards_m": background_spec["distance_behind_target_m"],
                                "policy": "wall lies beyond every mission and distractor board corner along its normal"}
    world.remove(original_board)
    for element in list(assets):
        if element.get("name") in ("task3_board_texture", "task3_board_material", "task3_board_mesh"):
            assets.remove(element)
    reports = {}
    for spec in config["targets"] + config.get("distractors", []):
        resolved = expand_target(config, spec)
        identity = spec["id"]
        texture_path = output/f"target_{identity}.png"
        tag_only = spec.get("tag_only", False)
        if tag_only:
            reports[str(identity)] = _tag_only_texture(resolved, texture_path)
            resolved["simulation"].update(board_width_m=resolved["tag"]["size_m"] * 1.25,
                                          board_height_m=resolved["tag"]["size_m"] * 1.25,
                                          board_center_down_m=0.0)
        else:
            reports[str(identity)] = _target_texture(resolved, texture_path)
        reports[str(identity)]["role"] = "target" if spec in config["targets"] else "distractor"
        _append_board(world, assets, resolved, texture_path, tag_only=tag_only)
    points = np.asarray([expand_target(config, s)["simulation"]["target_center_world_m"] for s in config["targets"]], float)
    center = np.mean(np.vstack([points, [0, 0, .3]]), axis=0)
    root.find("statistic").set("center", numbers(center))
    root.find("statistic").set("extent", str(max(3., np.linalg.norm(np.ptp(points, axis=0))+1.5)))
    # Preserve portable runtime loading even from Chinese Windows paths.
    runtime = copy.deepcopy(root)
    compiler = runtime.find("compiler")
    mesh_dir = Path(compiler.get("meshdir", str(model_root)))
    compiler.set("meshdir", "")
    compiler.set("texturedir", "")
    virtual_assets = {}
    for kind in ("mesh", "texture"):
        for index, node in enumerate(runtime.findall(f"asset/{kind}")):
            if "file" not in node.attrib:
                continue
            path = Path(node.get("file"))
            if not path.is_absolute():
                path = (mesh_dir if kind == "mesh" else output)/path
            name = f"multi_{kind}_{index}{path.suffix.lower()}"
            virtual_assets[name] = path.read_bytes()
            node.set("file", name)
    ET.indent(tree)
    tree.write(output/"scene.xml", encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_string(ET.tostring(runtime, encoding="unicode"), virtual_assets)
    provenance.update({"scene": str(output/"scene.xml"), "mode": "multi_fixed_targets", "moving_targets": False,
                       "mission_target_ids": [s["id"] for s in config["targets"]],
                       "distractor_ids": [s["id"] for s in config.get("distractors", [])],
                       "scoring_board_count": sum(not s.get("tag_only", False) for s in config["targets"] + config.get("distractors", [])),
                       "visual_only_distractor_tag_count": sum(s.get("tag_only", False) for s in config.get("distractors", [])),
                       "target_textures": reports,
                       "background_placement": background_placement,
                       "score_attribution": "nearest physical board only; misses recorded as ring 3 without inventing an actual board identity"})
    (output/"model_provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2), encoding="utf-8")
    return model, provenance
