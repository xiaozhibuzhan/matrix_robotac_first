"""Prepare a derived scene for the unchanged official MuJoCo executable."""
from __future__ import annotations

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .scene import numbers, quaternion, rpy_matrix
from .multi_config import expand_target

JOINT_NAMES = [f"{leg}_{joint}_JOINT" for leg in ("FAR", "FBL", "RAR", "RBL")
               for joint in ("ABAD", "HIP", "KNEE")]


def _records(node):
    return [(item.tag, dict(item.attrib)) for item in node]


def prepare_scene(scene_path, config, model_root):
    """Keep official sensor layout, inertias, joints and actuator semantics.

    No motor gains, control laws, or contact masks are replaced. The existing
    <=0.866 mm derived STL repair is retained and recorded by build_multi_scene.
    """
    path = Path(scene_path)
    tree = ET.parse(path)
    root = tree.getroot()
    original = ET.parse(Path(model_root)/"xgb.xml").getroot()
    if _records(root.find("sensor")) != _records(original.find("sensor")):
        raise ValueError("Official bridge requires the original sensor layout without additions")
    if _records(root.find("actuator")) != _records(original.find("actuator")):
        raise ValueError("Official torque actuator layout was changed")
    for source in original.findall(".//inertial") + original.findall(".//joint"):
        # All official joint names are unique; inertias are matched by body.
        if source.tag == "joint":
            derived = root.find(f".//joint[@name='{source.get('name')}']")
            if derived is None or derived.attrib != source.attrib:
                raise ValueError("Official joint limits, axes or friction changed")
    for source in original.findall(".//body"):
        inertia = source.find("inertial")
        if inertia is not None:
            derived = root.find(f".//body[@name='{source.get('name')}']/inertial")
            if derived is None or derived.attrib != inertia.attrib:
                raise ValueError("Official body inertia changed")
    base = root.find("worldbody/body[@name='base_link']")
    imu = base.find("site[@name='imu']")
    if (imu is None or not np.allclose(np.fromstring(imu.get("pos", "0 0 0"), sep=" "), 0)
            or not np.allclose(np.fromstring(imu.get("quat", "1 0 0 0"), sep=" "), [1, 0, 0, 0])):
        raise ValueError("RobotState-to-base mapping requires the official identity IMU site")
    base.set("pos", numbers(config["simulation"]["initial_position_m"]))
    rotation = rpy_matrix(config["simulation"]["initial_rpy_rad"])
    base.set("quat", numbers(quaternion(rotation)))
    # The official executable looks up this camera by name. Task RGB uses the
    # identical task3_camera; this alias only supports its native observer.
    camera = copy.deepcopy(base.find("camera[@name='task3_camera']"))
    camera.set("name", "this_camera")
    base.append(camera)
    # Keep the official home preset intact. The native executable starts with
    # mj_makeData/qpos0, NOT this keyframe: initial legs remain at official 0.
    # Its observer must not be used to load/reset a keyframe during acceptance.
    # The unmodified native viewer starts with MuJoCo's default visibility:
    # groups 0,1,2 enabled and 3,4,5 disabled. The render/referee mirror keeps
    # robot=3 (excluded from laser rays) and score surfaces=2 (hidden in RGB).
    # Only the native observer copy uses robot=0 and hidden score surfaces=5.
    # Rendering groups do not change geometry, contacts or actuator dynamics.
    native = copy.deepcopy(root)
    for geom in native.find("worldbody/body[@name='base_link']").iter("geom"):
        geom.set("group", "0")
    for geom in native.findall(".//geom[@group='2']"):
        geom.set("group", "5")
    # Four real surrounding walls can occlude a low external camera. Start
    # the native free camera above the open roof, as the Python overview does.
    initial = np.asarray(config["simulation"]["initial_position_m"], dtype=float)
    centers = np.array([expand_target(config, spec)["simulation"]["target_center_world_m"] for spec in config["targets"]])
    span = float(np.max(np.linalg.norm(centers-initial, axis=1)))
    height = float(config.get("appearance", {}).get("background", {}).get("height_m", 0))
    distance = max(8.5, 2.7*span, height+4.0)
    native.find("visual/global").set("elevation", "-65")
    native.find("visual/global").set("azimuth", "45")
    native.find("statistic").set("center", "0 0 0.3")
    native.find("statistic").set("extent", str(distance/1.5))
    result = path.with_name("official_scene.xml")
    ET.indent(native)
    ET.ElementTree(native).write(result, encoding="utf-8", xml_declaration=True)

    model = _compile_with_assets(root, path, model_root)
    if (model.nq, model.nv, model.nu, model.nsensordata) != (19, 18, 12, 52):
        raise ValueError("Unexpected official free-base / twelve-joint / sensor dimensions")
    return model, result


def _compile_with_assets(root, scene_path, model_root):
    """Compile an MJCF copy through VFS, including Windows Unicode paths."""
    path = Path(scene_path)
    runtime = copy.deepcopy(root)
    compiler = runtime.find("compiler")
    mesh_dir = Path(compiler.get("meshdir", str(model_root)))
    compiler.set("meshdir", "")
    compiler.set("texturedir", "")
    assets = {}
    for kind in ("mesh", "texture"):
        for index, node in enumerate(runtime.findall(f"asset/{kind}")):
            if "file" not in node.attrib:
                continue
            source = Path(node.get("file"))
            if not source.is_absolute():
                source = (mesh_dir if kind == "mesh" else path.parent)/source
            name = f"native_{kind}_{index}{source.suffix.lower()}"
            assets[name] = source.read_bytes()
            node.set("file", name)
    return mujoco.MjModel.from_xml_string(ET.tostring(runtime, encoding="unicode"), assets)
