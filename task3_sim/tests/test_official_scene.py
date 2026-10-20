"""Derived MJCF invariants on this host; no native controller is executed."""
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from task3_sim.multi_scene import build_multi_scene
from task3_sim.official_scene import _compile_with_assets, prepare_scene

PROJECT = Path(__file__).resolve().parents[2]


class OfficialSceneTests(unittest.TestCase):
    def test_native_startup_and_sensor_inertia_contract(self):
        config = json.loads((PROJECT/"task3_sim/config/multi_official.json").read_text(encoding="utf-8"))
        source = PROJECT/"src/robot_mujoco/zsibot_robots/xgb"
        with tempfile.TemporaryDirectory() as work:
            output = Path(work)
            build_multi_scene(config, source, output)
            model, path = prepare_scene(output/"scene.xml", config, source)
            self.assertEqual((model.nq, model.nv, model.nu, model.nsensordata), (19, 18, 12, 52))
            np.testing.assert_allclose(model.qpos0[:3], [0, 0, .65])
            np.testing.assert_allclose(model.qpos0[7:], 0)
            tree = ET.parse(path)
            original = ET.parse(source/"xgb.xml")
            self.assertEqual([dict(n.attrib) for n in tree.findall("sensor/*")],
                             [dict(n.attrib) for n in original.findall("sensor/*")])
            self.assertEqual(tree.find("keyframe/key").attrib, original.find("keyframe/key").attrib)
            self.assertIsNotNone(tree.find(".//camera[@name='this_camera']"))
            # Native default groups hid all robot geometry in 0.3.0. The
            # separate render/referee mirror must still exclude the shooter.
            native = _compile_with_assets(tree.getroot(), path, source)
            visibility = mujoco.MjvOption().geomgroup.copy()
            robot_ids = np.flatnonzero(model.geom_group == 3)
            self.assertGreater(len(robot_ids), 12)
            self.assertTrue(np.all(visibility[native.geom_group[robot_ids]]))
            self.assertTrue(np.all(model.geom_group[robot_ids] == 3))
            surfaces = [mujoco.mj_name2id(native, mujoco.mjtObj.mjOBJ_GEOM, f"task3_target_collision_{spec['id']}")
                        for spec in config["targets"]]
            self.assertFalse(np.any(visibility[native.geom_group[surfaces]]))
            self.assertTrue(np.all(model.geom_group[surfaces] == 2))
            for name in ("qpos0", "body_mass", "body_inertia", "body_ipos", "body_iquat",
                         "jnt_axis", "jnt_range", "dof_damping", "dof_frictionloss",
                         "geom_type", "geom_pos", "geom_quat", "geom_size", "geom_contype",
                         "geom_conaffinity", "geom_friction", "actuator_gainprm", "actuator_biasprm",
                         "actuator_ctrlrange", "sensor_type", "sensor_adr", "sensor_dim"):
                np.testing.assert_array_equal(getattr(native, name), getattr(model, name), err_msg=name)
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(native, camera)
            self.assertEqual(camera.elevation, -65)
            self.assertGreaterEqual(camera.distance, 8.5)
            # A reordered official layout must fail before launching binaries.
            bad = ET.parse(output/"scene.xml")
            sensor = bad.find("sensor")
            sensor.append(sensor[0]); sensor.remove(sensor[0])
            bad.write(output/"scene.xml")
            with self.assertRaises(ValueError):
                prepare_scene(output/"scene.xml", config, source)


if __name__ == "__main__":
    unittest.main()
