#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FlightTarget = Tuple[float, float, float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MinimalDropDemo(Node):
    """Small user example using only flight-control and drop interfaces."""

    def __init__(self) -> None:
        super().__init__("minimal_drop_demo")
        self.declare_parameter("drop_color", "red")
        self.declare_parameter("takeoff_z", 0.75)
        self.declare_parameter("drop_z", 0.55)
        self.declare_parameter("land_z", 0.20)

        self.current_pose: Optional[PoseStamped] = None
        self.current_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/flight_control/cmd_vel", 10)
        self.drop_pub = self.create_publisher(String, "/delivery/drop_cmd", 10)
        self.create_subscription(
            PoseStamped, "/flight_control/current_pose", self.on_current_pose, 10
        )
        self.create_subscription(String, "/delivery/package_state", self.on_package_state, 10)

    def on_current_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        q = msg.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_package_state(self, msg: String) -> None:
        if "event=" in msg.data:
            self.get_logger().info(f"投放装置状态：{msg.data}")

    def wait_for_pose(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose is not None:
                return True
        return False

    def fly_to(self, target: FlightTarget, timeout_sec: float = 12.0) -> bool:
        target_x, target_y, target_z, target_yaw = target
        deadline = time.monotonic() + timeout_sec
        rate_sec = 0.04

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.current_pose is None:
                time.sleep(rate_sec)
                continue

            pose = self.current_pose.pose
            ex = target_x - pose.position.x
            ey = target_y - pose.position.y
            ez = target_z - pose.position.z
            yaw_error = normalize_angle(target_yaw - self.current_yaw)

            world_vx = clamp(0.8 * ex, -0.28, 0.28)
            world_vy = clamp(0.8 * ey, -0.28, 0.28)
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)

            cmd = Twist()
            cmd.linear.x = cos_yaw * world_vx + sin_yaw * world_vy
            cmd.linear.y = -sin_yaw * world_vx + cos_yaw * world_vy
            cmd.linear.z = clamp(0.9 * ez, -0.25, 0.25)
            cmd.angular.z = clamp(1.2 * yaw_error, -0.6, 0.6)
            self.cmd_pub.publish(cmd)

            if math.hypot(ex, ey) < 0.06 and abs(ez) < 0.04 and abs(yaw_error) < 0.18:
                self.stop()
                return True
            time.sleep(rate_sec)

        self.stop()
        return False

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def run(self) -> None:
        self.get_logger().info("等待飞控当前位姿 /flight_control/current_pose...")
        if not self.wait_for_pose():
            self.get_logger().error("未收到当前位姿，请先启动仿真环境和 flight control。")
            return

        pose = self.current_pose.pose
        start_x = pose.position.x
        start_y = pose.position.y
        start_yaw = self.current_yaw
        takeoff_z = float(self.get_parameter("takeoff_z").value)
        drop_z = float(self.get_parameter("drop_z").value)
        land_z = float(self.get_parameter("land_z").value)
        drop_color = str(self.get_parameter("drop_color").value)

        self.get_logger().info("起飞到练习高度。")
        self.fly_to((start_x, start_y, takeoff_z, start_yaw))
        time.sleep(0.8)

        self.get_logger().info("下降一点，模拟靠近投放高度。")
        self.fly_to((start_x, start_y, drop_z, start_yaw))
        time.sleep(0.5)

        self.get_logger().info(f"发送投放命令：{drop_color}")
        self.drop_pub.publish(String(data=drop_color))
        time.sleep(1.0)

        self.get_logger().info("重新拉升后降落。")
        self.fly_to((start_x, start_y, takeoff_z, start_yaw))
        time.sleep(0.5)
        self.fly_to((start_x, start_y, land_z, start_yaw))
        self.stop()
        self.get_logger().info("最小投放示例完成。")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalDropDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
