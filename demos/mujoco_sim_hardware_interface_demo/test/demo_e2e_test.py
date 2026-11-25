#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from tf2_msgs.msg import TFMessage


class DemoProbe:
    def __init__(self):
        self.node = rclpy.create_node("mujoco_sim_hardware_interface_demo_e2e")
        self.clock_ns = []
        self.left_positions = []
        self.odom_x = []
        self.command_values = []
        self.sensor_arrays = []
        self.imu_arrays = []
        self.tf_received = False
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.node.create_subscription(Clock, "/clock", self._clock, clock_qos)
        self.node.create_subscription(
            JointState, "/joint_states", self._joints, qos_profile_sensor_data
        )
        self.node.create_subscription(Odometry, "/diff_drive_controller/odom", self._odom, 10)
        self.node.create_subscription(
            Float64,
            "/mujoco_sim_diff_drive/debug/joints/left_wheel_joint/command/velocity",
            self._command,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Float64MultiArray,
            "/mujoco_sim_diff_drive/debug/sensors/base_position_sensor",
            self.sensor_arrays.append,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Float64MultiArray,
            "/mujoco_sim_diff_drive/debug/imus/imu_sensor",
            self.imu_arrays.append,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.command = self.node.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )

    def _clock(self, message):
        self.clock_ns.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)

    def _joints(self, message):
        if "left_wheel_joint" in message.name:
            index = message.name.index("left_wheel_joint")
            if index < len(message.position):
                self.left_positions.append(message.position[index])

    def _odom(self, message):
        self.odom_x.append(message.pose.pose.position.x)

    def _command(self, message):
        self.command_values.append(message.data)

    def _tf(self, message):
        self.tf_received = self.tf_received or any(
            transform.header.frame_id == "world"
            and transform.child_frame_id == "mujoco_imu_pose"
            for transform in message.transforms
        )

    def ready(self):
        return (
            len(self.clock_ns) >= 2
            and self.left_positions
            and self.odom_x
            and self.sensor_arrays
            and self.imu_arrays
            and self.tf_received
            and self.command.get_subscription_count() > 0
        )

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def drive(self, duration):
        deadline = time.monotonic() + duration
        message = TwistStamped()
        message.twist.linear.x = 0.4
        while time.monotonic() < deadline:
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.04)

    def close(self):
        self.node.destroy_node()


def validate(probe):
    if any(later <= earlier for earlier, later in zip(probe.clock_ns, probe.clock_ns[1:])):
        raise AssertionError("/clock did not advance monotonically")
    if not probe.left_positions or max(probe.left_positions) - min(probe.left_positions) < 1.0:
        raise AssertionError("left wheel position did not update")
    if not probe.odom_x or max(probe.odom_x) < 0.2:
        raise AssertionError("odometry did not move forward")
    moving_commands = [value for value in probe.command_values if abs(value) > 0.1]
    if not moving_commands:
        raise AssertionError("no nonzero wheel command was accepted")
    if any(not math.isfinite(value) or abs(value) > 12.0 for value in probe.command_values):
        raise AssertionError("wheel command exceeded the Server range")
    if not probe.sensor_arrays or any(len(message.data) != 3 for message in probe.sensor_arrays):
        raise AssertionError("grouped generic sensor length is not 3")
    generic_samples = [message.data for message in probe.sensor_arrays]
    if any(not all(math.isfinite(value) for value in sample) for sample in generic_samples):
        raise AssertionError("grouped generic sensor contains a non-finite value")
    if max(sample[0] for sample in generic_samples) < 0.2:
        raise AssertionError("generic sensor is not ordered as configured x y z")
    if any(abs(sample[1]) > 0.2 or not 0.1 < sample[2] < 0.5 for sample in generic_samples):
        raise AssertionError("generic sensor y z values do not match configured order")
    if not probe.imu_arrays or any(len(message.data) != 13 for message in probe.imu_arrays):
        raise AssertionError("grouped IMU length is not 13")
    latest_imu = probe.imu_arrays[-1].data
    if any(not math.isfinite(value) for value in latest_imu):
        raise AssertionError("grouped IMU contains a non-finite value")
    if abs(sum(value * value for value in latest_imu[6:10]) - 1.0) > 1e-6:
        raise AssertionError("IMU orientation xyzw group is not normalized")
    if latest_imu[10] < 0.2 or abs(latest_imu[11]) > 0.2 or not 0.1 < latest_imu[12] < 0.5:
        raise AssertionError("IMU optional pose is not ordered orientation xyzw then position xyz")
    if not probe.tf_received:
        raise AssertionError("direct world -> mujoco_imu_pose TF was not received")


def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "173")
    os.environ.pop("DISPLAY", None)
    os.environ.pop("WAYLAND_DISPLAY", None)
    with tempfile.TemporaryFile(mode="w+") as launch_log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "mujoco_sim_hardware_interface_demo",
                "diff_drive.launch.py",
                "auto_command:=false",
            ],
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
        )
        probe = None
        try:
            rclpy.init()
            probe = DemoProbe()
            if not probe.spin_until(probe.ready, 15.0):
                raise AssertionError("demo topics or controllers were not ready within 15 seconds")
            probe.drive(2.5)
            if not probe.spin_until(lambda: max(probe.odom_x, default=0.0) > 0.2, 3.0):
                raise AssertionError("odometry did not pass the movement threshold")
            validate(probe)
        finally:
            if probe is not None:
                probe.close()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
            launch_log.seek(0)
            output = launch_log.read()
            sys.stderr.write(output)
            forbidden = (
                "simulation is unstable",
                "hardware activation failed",
                "Failed to activate controller",
            )
            if any(text in output for text in forbidden):
                raise AssertionError("demo launch log contains a runtime failure")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
