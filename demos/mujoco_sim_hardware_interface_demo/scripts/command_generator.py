#!/usr/bin/env python3

"""Publish one finite, bounded TwistStamped sequence for the demo launch."""

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node("mujoco_sim_bounded_command_generator")
    try:
        topic = node.declare_parameter(
            "topic", "/diff_drive_controller/cmd_vel"
        ).value
        linear_x = node.declare_parameter("linear_x", 0.4).value
        angular_z = node.declare_parameter("angular_z", 0.2).value
        duration_s = node.declare_parameter("duration_s", 3.0).value
        rate_hz = node.declare_parameter("rate_hz", 20.0).value
        wait_timeout_s = node.declare_parameter("wait_timeout_s", 15.0).value

        numeric = (linear_x, angular_z, duration_s, rate_hz, wait_timeout_s)
        if not all(isinstance(value, (float, int)) and math.isfinite(value) for value in numeric):
            raise ValueError("command parameters must be finite numbers")
        if duration_s < 0.0 or rate_hz <= 0.0 or wait_timeout_s <= 0.0:
            raise ValueError("duration must be non-negative and rate/timeouts must be positive")

        publisher = node.create_publisher(TwistStamped, topic, 10)
        wait_deadline = time.monotonic() + wait_timeout_s
        while publisher.get_subscription_count() == 0 and time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"no command subscriber appeared on {topic!r}")

        message = TwistStamped()
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        interval_s = 1.0 / float(rate_hz)
        command_deadline = time.monotonic() + float(duration_s)
        while rclpy.ok() and time.monotonic() < command_deadline:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(interval_s)

        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = 0.0
        message.twist.angular.z = 0.0
        for _ in range(5):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(interval_s)
        node.get_logger().info("bounded demonstration command completed")
        return 0
    except (RuntimeError, ValueError) as exception:
        node.get_logger().error(str(exception))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
