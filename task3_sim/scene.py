"""Generate a separate MJCF by reading the supplied xgb model, never editing it."""

from __future__ import annotations

import hashlib
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

from .geometry import camera_extrinsics, normalized, rotation3, vector3
from .mesh import prepare_stl
from .appearance import photo_target_texture, validate_appearance, write_image_texture
from .scoring import validate_ring_ellipses


def numbers(values) -> str:
    return " ".join(f"{float(x):.12g}" for x in np.asarray(values).ravel())


def quaternion(rotation: np.ndarray) -> np.ndarray:
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(rotation).reshape(9))
    return quat


def rpy_matrix(rpy) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def validate_config(config: dict) -> None:
    camera, tag, sim = config["camera"], config["tag"], config["simulation"]
    width, height = int(camera["width"]), int(camera["height"])
    if min(width, height) < 64 or max(width, height) > 4096:
        raise ValueError("camera size must be between 64 and 4096 pixels")
    # fovy-based renderer: reject unsupported intrinsics rather than silently
    # using a different projection from the detector.
    if not np.allclose([camera["fx"], camera["fy"]], [camera["fy"], camera["fx"]]) or not np.allclose(
        [camera["cx"], camera["cy"]], [width/2, height/2]
    ) or camera["fy"] <= 0:
        raise ValueError("milestone renderer requires fx=fy>0 and a centered principal point")
    camera_extrinsics(camera)
    vector3(config["laser"]["origin_base_m"])
    normalized(config["laser"]["direction_base"])
    if tag["family"] != "tag36h11" or not 0 <= int(tag["id"]) < 587:
        raise ValueError("texture generator supports tag36h11 IDs 0..586")
    if not 0 < float(tag["size_m"]) < sim["board_width_m"]:
        raise ValueError("tag must fit the board")
    if int(sim["ammo_count"]) < 0:
        raise ValueError("ammo_count must be nonnegative")
    radii = np.asarray(sim["ring_radii_m"], dtype=float)
    if radii.shape != (5,) or not np.all(np.isfinite(radii)) or np.any(radii <= 0) or np.any(np.diff(radii) <= 0):
        raise ValueError("provide five increasing positive radii for rings 10,9,8,7,6")
    offset = vector3(tag["bullseye_offset_tag_m"])
    if abs(offset[2]) > 1e-10:
        raise ValueError("milestone tag and bullseye must be coplanar")
    if sim["board_width_m"]/2 < radii[-1] or sim["board_height_m"]/2-abs(sim["board_center_down_m"]) < radii[-1]:
        raise ValueError("all scoring rings must fit on the board")
    for key in ("yaw_kp", "pitch_kp", "max_yaw_rate_rad_s", "max_pitch_rate_rad_s", "aim_tolerance_m", "stable_duration_s", "max_frame_age_s", "max_feedback_age_s", "max_pitch_rad"):
        if not np.isfinite(config["control"][key]) or config["control"][key] <= 0:
            raise ValueError(f"control.{key} must be finite and positive")
    for key in ("frame_rate_hz", "integration_step_s", "velocity_time_constant_s", "command_timeout_s"):
        if not np.isfinite(sim[key]) or sim[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    for value in (sim["initial_position_m"], sim["initial_rpy_rad"], sim["target_center_world_m"]):
        vector3(value)
    if abs(sim["initial_rpy_rad"][0]) > 1e-10:
        raise ValueError("the kinematic milestone holds roll at zero")
    if abs(sim["initial_rpy_rad"][1]) > config["control"]["max_pitch_rad"]:
        raise ValueError("initial pitch is outside the allowed attitude range")
    validate_ring_ellipses(sim)
    validate_appearance(config)


def make_target_texture(config: dict, path: Path) -> None:
    """A real tag36h11 bitmap at the configured black-square metric size."""
    sim, tag = config["simulation"], config["tag"]
    ppm = 2400
    width, height = sim["board_width_m"], sim["board_height_m"]
    center_down = sim["board_center_down_m"]
    image = np.full((round(height*ppm), round(width*ppm), 3), 245, np.uint8)

    def pixel(x, y):
        return round((x+width/2)*ppm), round((y-center_down+height/2)*ppm)

    center = pixel(0, 0)
    cv2.circle(image, center, round(sim["ring_radii_m"][-1]*ppm), (55, 145, 65), -1)
    for radius, label in zip(sim["ring_radii_m"], range(10, 5, -1)):
        cv2.circle(image, center, round(radius*ppm), (255, 255, 255), 2, cv2.LINE_AA)
        if label != 10:
            cv2.putText(image, str(label), pixel(radius-0.012, 0.005), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(image, "10", (center[0]-17, center[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
    offset = vector3(tag["bullseye_offset_tag_m"])
    size = round(tag["size_m"]*ppm)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), int(tag["id"]), size)
    # OpenCV's AprilTag dictionary image has object X/Y rotated 180 degrees
    # relative to AprilTag3's metric pose convention. Make rendered +Y point
    # down toward the bullseye, as declared by bullseye_offset_tag_m.
    marker = cv2.rotate(marker, cv2.ROTATE_180)
    x0, y0 = pixel(-offset[0]-tag["size_m"]/2, -offset[1]-tag["size_m"]/2)
    if x0 < 12 or y0 < 12 or x0+size+12 > image.shape[1] or y0+size+12 > image.shape[0]:
        raise ValueError("tag and quiet border do not fit board")
    image[y0:y0+size, x0:x0+size] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    # imencode supports Windows paths containing Chinese characters.
    path.write_bytes(cv2.imencode(".png", image)[1].tobytes())


def build_scene(config: dict, model_root: Path, output: Path) -> tuple[mujoco.MjModel, dict]:
    validate_config(config)
    model_root = model_root.resolve()
    source = model_root / "xgb.xml"
    if not source.is_file():
        raise FileNotFoundError(f"Official model missing: {source}; use --model-root with the directory containing xgb.xml")
    output.mkdir(parents=True, exist_ok=True)
    texture_path = output / "target.png"
    appearance_report = {}
    target_report = photo_target_texture(config, texture_path)
    if target_report is None:
        make_target_texture(config, texture_path)
    else:
        appearance_report["target"] = target_report
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "task3_independent_single_target")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    asset_root = (model_root / compiler.get("meshdir", "")).resolve()
    compiler.set("meshdir", str(asset_root))
    compiler.set("texturedir", str(output.resolve()))
    world = root.find("worldbody")
    base = world.find("body[@name='base_link']")
    if base is None or base.find("freejoint") is None:
        raise ValueError("expected original xgb base_link with a freejoint")
    for parent in base.iter():
        for camera in list(parent.findall("camera")):
            parent.remove(camera)
    # The collision ray ignores the shooter's geometry (group 3), while the
    # original body shapes, inertia, joints and actuators remain in this model.
    for geom in base.iter("geom"):
        geom.set("group", "3")
    camera = config["camera"]
    rotation, position = camera_extrinsics(camera)
    camera_gl_rotation = rotation @ np.diag([1.0, -1.0, -1.0])
    ET.SubElement(base, "camera", name="task3_camera", pos=numbers(position),
                  quat=numbers(quaternion(camera_gl_rotation)),
                  fovy=str(math.degrees(2*math.atan(camera["height"]/(2*camera["fy"])))),
                  resolution=f"{camera['width']} {camera['height']}")
    ET.SubElement(base, "site", name="task3_emitter", pos=numbers(config["laser"]["origin_base_m"]), size="0.008", rgba="1 0.1 0.1 1")
    ET.SubElement(root, "option", timestep=str(config["simulation"]["integration_step_s"]))
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(camera["width"]), offheight=str(camera["height"]), azimuth="45", elevation="-20")
    ET.SubElement(visual, "map", znear="0.01", zfar="100")
    lighting = config.get("appearance", {}).get("lighting", {})
    ambient, diffuse = lighting.get("ambient", 0.5), lighting.get("diffuse", 0.5)
    ET.SubElement(visual, "headlight", ambient=numbers([ambient]*3), diffuse=numbers([diffuse]*3), specular="0 0 0")
    ET.SubElement(root, "statistic", center="1.2 0 0.4", extent="3")
    ET.SubElement(world, "light", pos="0 0 4", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="task3_floor", type="plane", size="5 5 0.1", rgba="0.28 0.30 0.33 1", group="0")
    assets = root.find("asset")
    ET.SubElement(assets, "texture", name="task3_sky", type="skybox", builtin="gradient", rgb1="0.12 0.16 0.24", rgb2="0.40 0.45 0.55", width="256", height="1536")
    ET.SubElement(assets, "texture", name="task3_board_texture", type="2d", file=str(texture_path.resolve()))
    ET.SubElement(assets, "material", name="task3_board_material", texture="task3_board_texture", texuniform="false", emission="1", specular="0", shininess="0")
    sim = config["simulation"]
    # Board axes: right/down/into board. A yaw rotates the whole target plane.
    board_rotation = rpy_matrix([0, 0, sim["target_yaw_rad"]]) @ np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    board = ET.SubElement(world, "body", name="task3_target", pos=numbers(sim["target_center_world_m"]), quat=numbers(quaternion(board_rotation)))
    half_w, half_h, down = sim["board_width_m"]/2, sim["board_height_m"]/2, sim["board_center_down_m"]
    # Thin watertight visual mesh, explicit UVs: the bitmap is genuinely rendered
    # in perspective by MuJoCo, never pasted into the camera observation.
    front = [[-half_w, down-half_h, 0], [half_w, down-half_h, 0], [half_w, down+half_h, 0], [-half_w, down+half_h, 0]]
    vertices = front + [[x,y,0.012] for x,y,_ in front]
    faces = [0,2,1, 0,3,2, 4,5,6, 4,6,7, 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7]
    ET.SubElement(assets, "mesh", name="task3_board_mesh", vertex=numbers(vertices), face=" ".join(map(str, faces)), texcoord=numbers([[0,0],[1,0],[1,1],[0,1]]*2))
    ET.SubElement(board, "geom", name="task3_board_visual", type="mesh", mesh="task3_board_mesh", material="task3_board_material", contype="0", conaffinity="0", group="1")
    # Ray queries skip alpha-zero objects. Hide group 2 in the renderer instead.
    ET.SubElement(board, "geom", name="task3_target_collision", type="box", pos=numbers([0,down,0.006]), size=numbers([half_w,half_h,0.006]), rgba="0.2 0.2 0.2 1", group="2")
    background = config.get("appearance", {}).get("background")
    if background is not None:
        background_path = output / "background.png"
        appearance_report["background"] = write_image_texture(background, background_path)
        ET.SubElement(assets, "texture", name="task3_background_texture", type="2d", file=str(background_path.resolve()))
        ET.SubElement(assets, "material", name="task3_background_material", texture="task3_background_texture",
                      texuniform="false", emission="1", specular="0", shininess="0")
        w, h = background["width_m"]/2, background["height_m"]/2
        depth = background["distance_behind_target_m"]
        # Explicit UVs on a watertight mesh render the actual supplied image in
        # both the camera and the viewer. Group 1 is excluded by the referee.
        bg_front = [[-w,-h,depth],[w,-h,depth],[w,h,depth],[-w,h,depth]]
        bg_vertices = bg_front + [[x,y,z+0.02] for x,y,z in bg_front]
        tile_x, tile_y = background.get("tile", [1,1])
        ET.SubElement(assets, "mesh", name="task3_background_mesh", vertex=numbers(bg_vertices),
                      face=" ".join(map(str, faces)), texcoord=numbers([[0,0],[tile_x,0],[tile_x,tile_y],[0,tile_y]]*2))
        ET.SubElement(board, "geom", name="task3_background_visual", type="mesh", mesh="task3_background_mesh",
                      material="task3_background_material", contype="0", conaffinity="0", group="1")
    for index, obstacle in enumerate(sim.get("obstacles", [])):
        ET.SubElement(world, "geom", name=f"task3_obstacle_{index}", type="box", pos=numbers(obstacle["position_m"]), size=numbers(obstacle["half_size_m"]), rgba="0.5 0.3 0.1 1", group="0")
    # Native MuJoCo fopen cannot open all Windows Unicode paths. A virtual file
    # system also makes the same read-only import independent of host paths.
    runtime_root = copy.deepcopy(root)
    runtime_root.find("compiler").set("meshdir", "")
    runtime_root.find("compiler").set("texturedir", "")
    virtual_assets = {}
    for index, texture in enumerate(runtime_root.findall("asset/texture")):
        if "file" in texture.attrib:
            texture_source = Path(texture.get("file"))
            virtual_name = f"task3_texture_{index}{texture_source.suffix.lower()}"
            virtual_assets[virtual_name] = texture_source.read_bytes()
            texture.set("file", virtual_name)
    mesh_reports = {}
    for index, mesh in enumerate(runtime_root.findall("asset/mesh")):
        if "file" in mesh.attrib:
            original_file = asset_root / mesh.attrib["file"]
            virtual_name = f"official_mesh_{index}{original_file.suffix.lower()}"
            raw = original_file.read_bytes()
            if original_file.suffix.lower() == ".stl":
                raw, report = prepare_stl(raw)
                mesh_reports[original_file.name] = report
                if report["changed"]:
                    derived_path = output / f"derived_{original_file.name}"
                    derived_path.write_bytes(raw)
                    # The exported scene also references the derived mesh.
                    root.find(f"asset/mesh[@name='{mesh.attrib['name']}']").set("file", str(derived_path.resolve()))
            virtual_assets[virtual_name] = raw
            mesh.set("file", virtual_name)
    xml_path = output / "scene.xml"
    ET.indent(tree)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_string(ET.tostring(runtime_root, encoding="unicode"), virtual_assets)
    referenced = [source] + sorted(asset_root.glob("*.STL")) + sorted(asset_root.glob("*.stl"))
    manifest = {str(p.relative_to(model_root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in referenced}
    provenance = {"source_model": str(source), "source_files_sha256": manifest,
                  "mesh_preparation": mesh_reports,
                  "adapter": "kinematic_base_with_velocity_lag", "official_files_modified": False,
                  "scene": str(xml_path), "target_dimensions": "synthetic, not official competition dimensions"}
    if appearance_report:
        provenance["appearance"] = appearance_report
    (output / "model_provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2), encoding="utf-8")
    return model, provenance
