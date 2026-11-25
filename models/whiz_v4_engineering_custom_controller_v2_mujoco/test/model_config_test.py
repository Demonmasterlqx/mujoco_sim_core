#!/usr/bin/env python3
import ast
import importlib.util
import math
import struct
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from launch.actions import EmitEvent
from launch.events import Shutdown


PACKAGE_ROOT = Path(sys.argv[1]).resolve()
sys.argv[:] = [sys.argv[0]]
EXPECTED_JOINTS = [f"j{index}" for index in range(7)]
EXPECTED_RANGES = [
    [-0.3, 3.14159],
    [-0.1, 1.7],
    [-0.1, 3.14159],
    [-1.4, 3.14159],
    [-0.5, 1.7],
    [-1.05, 1.9],
    [-3.14159, 3.14159],
]
EXPECTED_J6_MASS = 0.00666677140172814


def _numbers(value):
    return tuple(float(item) for item in value.split())


def _subtract(first, second):
    return tuple(left - right for left, right in zip(first, second))


def _norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def _primitive_distance(point, geom):
    geom_type = geom.attrib.get("type", "capsule")
    size = _numbers(geom.attrib["size"])
    if geom_type == "sphere":
        return _norm(_subtract(point, _numbers(geom.attrib["pos"]))) - size[0]
    if geom_type == "capsule":
        endpoints = _numbers(geom.attrib["fromto"])
        first = endpoints[:3]
        second = endpoints[3:]
        segment = _subtract(second, first)
        relative = _subtract(point, first)
        length_squared = sum(value * value for value in segment)
        alpha = max(
            0.0,
            min(1.0, sum(left * right for left, right in zip(relative, segment)) / length_squared),
        )
        closest = tuple(first[index] + alpha * segment[index] for index in range(3))
        return _norm(_subtract(point, closest)) - size[0]
    center = _numbers(geom.attrib.get("pos", "0 0 0"))
    relative = _subtract(point, center)
    if geom_type == "box":
        offset = tuple(abs(relative[index]) - size[index] for index in range(3))
        outside = _norm(tuple(max(value, 0.0) for value in offset))
        return outside + min(max(offset), 0.0)
    if geom_type == "cylinder":
        radial = math.hypot(relative[0], relative[1]) - size[0]
        axial = abs(relative[2]) - size[1]
        outside = math.hypot(max(radial, 0.0), max(axial, 0.0))
        return outside + min(max(radial, axial), 0.0)
    raise AssertionError(f"unsupported collision primitive type: {geom_type}")


def _sample_binary_stl_vertices(path, maximum_triangles=5000):
    content = path.read_bytes()
    triangle_count = struct.unpack_from("<I", content, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(content) != expected_size:
        raise AssertionError(f"{path} is not a supported binary STL")
    stride = max(1, math.ceil(triangle_count / maximum_triangles))
    vertices = []
    for triangle in range(0, triangle_count, stride):
        triangle_offset = 84 + triangle * 50
        for vertex in range(3):
            vertices.append(struct.unpack_from("<3f", content, triangle_offset + 12 + vertex * 12))
    return vertices


class ModelConfigTest(unittest.TestCase):
    def test_xacro_exports_effort_commands_and_finite_limits(self):
        result = subprocess.run(
            ["xacro", str(PACKAGE_ROOT / "config" / "whiz_v4_engineering_custom_controller_v2.urdf.xacro")],
            check=True,
            capture_output=True,
            text=True,
        )
        robot = ET.fromstring(result.stdout)
        control = robot.find("ros2_control")
        self.assertIsNotNone(control)
        mapped_joints = control.findall("joint")
        self.assertEqual([joint.attrib["name"] for joint in mapped_joints], EXPECTED_JOINTS)
        for joint in mapped_joints:
            parameters = {item.attrib["name"]: (item.text or "").strip() for item in joint.findall("param")}
            self.assertEqual(parameters["actuator_effort"], f"{joint.attrib['name']}_actuator")
            self.assertNotIn("actuator_position", parameters)
            self.assertEqual(parameters["sensor_position"], f"{joint.attrib['name']}_pos")
            self.assertEqual(parameters["sensor_velocity"], f"{joint.attrib['name']}_vel")
            self.assertEqual(parameters["sensor_effort"], f"{joint.attrib['name']}_tor")

        urdf_joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
        for name in EXPECTED_JOINTS:
            limit = urdf_joints[name].find("limit")
            self.assertIsNotNone(limit)
            self.assertGreater(float(limit.attrib["upper"]), float(limit.attrib["lower"]))
            self.assertEqual(float(limit.attrib["effort"]), 10.0)
            self.assertEqual(float(limit.attrib["velocity"]), 3.0)

    def test_both_urdfs_match_mjcf_limits_and_j6_mass(self):
        model = ET.parse(
            PACKAGE_ROOT / "whiz_v4_engineering_custom_controller_v2.xml"
        ).getroot()
        model_joints = {
            joint.attrib["name"]: joint
            for joint in model.findall(".//joint")
            if "name" in joint.attrib
        }
        model_ranges = [
            [float(value) for value in model_joints[name].attrib["range"].split()]
            for name in EXPECTED_JOINTS
        ]
        self.assertEqual(model_ranges, EXPECTED_RANGES)
        model_j6 = model.find(".//body[@name='j6_link']/inertial")
        self.assertIsNotNone(model_j6)
        self.assertEqual(float(model_j6.attrib["mass"]), EXPECTED_J6_MASS)

        for relative_path in [
            "whiz_v4_engineering_custom_controller_v2.urdf",
            "config/whiz_v4_engineering_custom_controller_v2.urdf",
        ]:
            robot = ET.parse(PACKAGE_ROOT / relative_path).getroot()
            urdf_joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
            urdf_ranges = [
                [
                    float(urdf_joints[name].find("limit").attrib["lower"]),
                    float(urdf_joints[name].find("limit").attrib["upper"]),
                ]
                for name in EXPECTED_JOINTS
            ]
            self.assertEqual(urdf_ranges, EXPECTED_RANGES, relative_path)
            urdf_j6_mass = robot.find("./link[@name='j6_link']/inertial/mass")
            self.assertIsNotNone(urdf_j6_mass, relative_path)
            self.assertEqual(
                float(urdf_j6_mass.attrib["value"]), EXPECTED_J6_MASS, relative_path
            )

    def test_controller_configuration_matches_torque_contract(self):
        with (PACKAGE_ROOT / "config" / "controllers.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        manager = config["controller_manager"]["ros__parameters"]
        self.assertEqual(manager["update_rate"], 200)
        self.assertEqual(
            manager["arm_controller"]["type"],
            "effort_controllers/JointGroupEffortController",
        )
        arm = config["arm_controller"]["ros__parameters"]
        self.assertEqual(arm["joints"], EXPECTED_JOINTS)
        self.assertNotIn("command_interfaces", arm)
        self.assertNotIn("state_interfaces", arm)

    def test_moving_collision_proxies_cover_visual_mesh_samples(self):
        model = ET.parse(
            PACKAGE_ROOT / "whiz_v4_engineering_custom_controller_v2.xml"
        ).getroot()
        for body_name in ["base_link", *(f"{link}_link" for link in EXPECTED_JOINTS)]:
            body = model.find(f".//body[@name='{body_name}']")
            self.assertIsNotNone(body, body_name)
            collision_geoms = [
                geom
                for geom in body.findall("geom")
                if geom.attrib.get("name", "").startswith(f"{body_name}_collision")
            ]
            self.assertTrue(collision_geoms, body_name)
            errors = []
            for point in _sample_binary_stl_vertices(
                PACKAGE_ROOT / "meshes" / f"{body_name}.STL"
            ):
                errors.append(
                    max(
                        0.0,
                        min(_primitive_distance(point, geom) for geom in collision_geoms),
                    )
                )
            within_ten_mm = sum(error <= 0.01 for error in errors) / len(errors)
            within_twenty_mm = sum(error <= 0.02 for error in errors) / len(errors)
            self.assertGreaterEqual(within_ten_mm, 0.90, body_name)
            self.assertGreaterEqual(within_twenty_mm, 0.95, body_name)
            self.assertLessEqual(max(errors), 0.045, body_name)

    def test_launch_file_is_valid_python_and_uses_home_keyframe(self):
        launch_path = PACKAGE_ROOT / "launch" / "launch_hardware_interface.py"
        source = launch_path.read_text(encoding="utf-8")
        ast.parse(source, filename=str(launch_path))
        self.assertIn('"--initial-keyframe"', source)
        self.assertIn('"home"', source)
        self.assertIn('"--headless"', source)
        self.assertIn('"arm_controller"', source)
        self.assertIn('"--inactive"', source)
        self.assertIn('"/clock"', source)
        self.assertIn('"rosgraph_msgs/msg/Clock"', source)
        self.assertIn('FindExecutable(name="timeout")', source)
        self.assertIn('"10s"', source)
        self.assertIn('"switch_controllers"', source)

    def test_clock_waiter_timeout_requests_launch_shutdown(self):
        launch_path = PACKAGE_ROOT / "launch" / "launch_hardware_interface.py"
        spec = importlib.util.spec_from_file_location("whiz_launch_for_test", launch_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        event = type("ProcessExit", (), {"returncode": 124})()
        actions = module._continue_or_shutdown(
            event, None, process_name="initial hardware-state waiter"
        )
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], EmitEvent)
        self.assertIsInstance(actions[0].event, Shutdown)
        self.assertIn("return code 124", actions[0].event.reason)


if __name__ == "__main__":
    unittest.main()
