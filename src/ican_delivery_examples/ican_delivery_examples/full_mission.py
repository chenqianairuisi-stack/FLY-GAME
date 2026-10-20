"""Closed-loop practice mission. No referee, teleportation or fake detections."""
import json
import math
from pathlib import Path
import re
import time
from datetime import datetime

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as FlightPath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .mission_core import angle, body_velocity, make_route, parse_order, yaw


def clamp(value, limit):
    return max(-limit, min(limit, value))


class DeliveryMission(Node):
    def __init__(self):
        super().__init__("delivery_mission")
        defaults = dict(drop_order="1234", home=[3.07, -2.38],
                        windows=[1.925, -.3275, 1.925, -2.3725],
                        drop_points=[1.12, -.15, .02, -.67, .02, -2.03, 1.12, -2.53],
                        east_lane_x=2.8, west_lane_x=1.05, crossing_z=.74,
                        observe_z=1.27, land_z=.2,
                        # Simulator world displacement is measured at 2x
                        # cmd_vel. Cap the complete 3D velocity at 0.5 m/s.
                        max_speed=.5, sim_velocity_gain=2.,
                        cruise_speed=.25, crossing_speed=.25, delivery_speed=.25,
                        throw_z=1.05, drop4_throw_z=.74, throw_lead=.22,
                        waypoint_timeout=90., detection_timeout=35., sensor_timeout=3.,
                        lidar_enabled=True, obstacle_margin=.025, report_dir="demo_runs")
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {k: self.get_parameter(k).value for k in defaults}
        self.order = parse_order(self.config["drop_order"])
        for key, size in (("home", 2), ("windows", 4), ("drop_points", 8)):
            if len(self.config[key]) != size or not all(math.isfinite(v) for v in self.config[key]):
                raise ValueError(f"参数 {key} 需要 {size} 个有限坐标")
        self.config["windows"] = list(zip(self.config["windows"][::2], self.config["windows"][1::2]))
        self.config["drop_points"] = list(zip(self.config["drop_points"][::2], self.config["drop_points"][1::2]))
        if not .64 <= self.config["crossing_z"] <= .77:
            raise ValueError("当前场景穿窗高度必须在 0.64–0.77 m，兼顾脚架和旋翼尺寸")
        if not .64 <= self.config["drop4_throw_z"] <= .77:
            raise ValueError("第四点投掷后直接穿窗，高度必须在 0.64–0.77 m")
        for key in ("max_speed", "sim_velocity_gain", "cruise_speed", "crossing_speed", "delivery_speed", "waypoint_timeout", "detection_timeout", "sensor_timeout"):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"{key} 必须是正数")
        self.route = make_route(self.config, self.order)
        self.pose = None
        self.pose_wall = 0.
        self.lidar_wall = 0.
        self.package_wall = 0.
        self.points = np.empty((0, 3))
        self.carried = set()
        self.dropped = set()
        self.started = False
        self.finished = False
        self.exit_code = 1
        self.index = 0
        self.step_started = 0.
        self.stable_since = None
        self.blocked_since = None
        self.request = None
        self.detection = None
        self.last_request = -math.inf
        self.last_trace = -math.inf
        self.throw_ready = False
        self.throw_results = {}
        self.departure_until = -math.inf
        self.departure_velocity = (0., 0.)
        self.pending_throws = {}
        self.previous_pose = None
        self.measured_velocity = [0., 0., 0.]
        self.motion_samples = []
        self.crossed_second = False
        self.created_wall = time.monotonic()
        self.session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output = Path(self.config["report_dir"]).expanduser().resolve() / self.session
        self.output.mkdir(parents=True, exist_ok=True)
        self.report = dict(result="waiting", order=self.order, config=self.config,
                           events=[], detections={}, drops=[], throws={}, trajectory=[])
        self.cmd = self.create_publisher(Twist, "/flight_control/cmd_vel", 10)
        self.drop = self.create_publisher(String, "/delivery/throw_cmd", 10)
        self.detect_request = self.create_publisher(String, "/delivery/detection_request", 10)
        self.status = self.create_publisher(String, "/delivery/mission_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_pub = self.create_publisher(FlightPath, "/delivery/flight_path", 1)
        self.plan_pub = self.create_publisher(FlightPath, "/delivery/planned_path", 1)
        self.label_pub = self.create_publisher(Marker, "/delivery/mission_label", 1)
        self.flight_path = FlightPath()
        self.flight_path.header.frame_id = "odom"
        self.create_subscription(PoseStamped, "/flight_control/current_pose", self.on_pose, 10)
        self.create_subscription(String, "/delivery/package_state", self.on_package, 10)
        self.create_subscription(PointCloud2, "/uav/lidar/points", self.on_lidar, qos_profile_sensor_data)
        self.create_subscription(String, "/delivery/detections", self.on_detection, 10)
        self.create_subscription(String, "/delivery/mission_command", self.on_command, 10)
        self.create_subscription(String, "/delivery/throw_state", self.on_throw, 10)
        # Wall-clock timer also runs while Gazebo is starting or paused.
        self.timer = self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"等待仿真和传感器；投放顺序 {self.order}；记录 {self.output}")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, msg):
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z, msg.pose.orientation.w)):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if self.previous_pose:
                old_stamp, old = self.previous_pose
                dt = stamp-old_stamp
                if dt > 1e-5:
                    self.measured_velocity = [(v-o)/dt for v, o in zip((p.x,p.y,p.z), old)]
            self.previous_pose = (stamp, [p.x,p.y,p.z])
            self.motion_samples.append([stamp, p.x, p.y, p.z, *self.measured_velocity])
            self.report["peak_speed_mps"] = max(self.report.get("peak_speed_mps", 0.), math.hypot(*self.measured_velocity[:2]))
            self.report["peak_3d_speed_mps"] = max(self.report.get("peak_3d_speed_mps", 0.), math.hypot(*self.measured_velocity))
            self.motion_samples = self.motion_samples[-300:]
            self.pose = msg.pose
            self.pose_wall = time.monotonic()

    def on_package(self, msg):
        for name in ("carried", "dropped"):
            match = re.search(rf"\b{name}=([^\s;]+)", msg.data)
            if match:
                setattr(self, name, set(match[1].split(",")) - {"none", ""})
        self.package_wall = time.monotonic()

    def on_throw(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if data.get("status") == "ready":
            self.throw_ready = True
        elif data.get("token", "").startswith(self.session + "_"):
            token = data["token"]
            self.report["throws"].setdefault(token, {})[data["status"]] = data
            if data["status"] == "released":
                self.throw_results[token] = data
            self.save()

    def on_lidar(self, msg):
        if time.monotonic()-self.lidar_wall < .12:
            return
        values = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if isinstance(values, np.ndarray) and values.dtype.names:
            self.points = np.column_stack([values[name] for name in ("x", "y", "z")]).astype(float)
        else:
            self.points = np.array(list(values), dtype=float).reshape(-1, 3)
        # Sensor origin -> aircraft base, using supplied static TF.
        self.points += np.array([.12, 0., .18])
        self.lidar_wall = time.monotonic()

    def on_detection(self, msg):
        try:
            result = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if self.detection is None and self.request and all(result.get(k) == self.request[k] for k in ("window", "token")):
            self.detection = result

    def on_command(self, msg):
        if msg.data in ("abort", "stop"):
            self.finish(False, "用户停止任务，已发送零速度并悬停；重新运行需重启仿真")

    def save(self):
        path = self.output / "report.json"
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(path)

    def event(self, message, **extra):
        data = dict(sim_time=round(self.now(), 3), message=message, **extra)
        self.report["events"].append(data)
        self.get_logger().info(message)
        if rclpy.ok():
            self.status.publish(String(data=json.dumps(data, ensure_ascii=False)))
        self.save()

    def hold(self):
        if rclpy.ok():
            self.cmd.publish(Twist())

    def publish_velocity(self, command):
        # Limit the vector norm, not each axis independently: simultaneous
        # translation and descent must also obey the actual flight limit.
        linear = command.linear
        speed = math.hypot(linear.x, linear.y, linear.z)
        limit = self.config["max_speed"] / self.config["sim_velocity_gain"]
        if speed > limit:
            scale = limit / speed
            linear.x *= scale
            linear.y *= scale
            linear.z *= scale
        self.cmd.publish(command)

    def publish_label(self, text, success=True):
        if not rclpy.ok():
            return
        label = Marker()
        label.header.frame_id = "odom"
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns, label.id, label.type, label.action = "mission", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = 2.8, -1.35, 2.
        label.pose.orientation.w = 1.
        label.scale.z = .12
        label.color.r, label.color.g, label.color.b, label.color.a = (.3, 1., .5, 1.) if success else (1., .2, .2, 1.)
        label.text = text
        self.label_pub.publish(label)

    def finish(self, success, reason):
        if self.finished:
            return
        self.hold()
        if rclpy.ok():
            self.detect_request.publish(String(data="cancel"))
            self.drop.publish(String(data="cancel"))
        self.finished = True
        self.exit_code = 0 if success else 1
        self.report["result"] = "completed" if success else "failed"
        self.report["reason"] = reason
        self.report["elapsed_sim_sec"] = self.now()-self.report.get("started_sim", self.now())
        self.report["elapsed_wall_sec"] = time.monotonic()-self.created_wall
        self.publish_label("COMPLETED - 4/4 delivered" if success else "STOPPED - see report.json", success)
        self.event(reason, result=self.report["result"], report=str(self.output / "report.json"))

    def advance(self):
        step = self.route[self.index]
        self.event(f"完成 {step['name']}", step=self.index)
        self.index += 1
        self.step_started = self.now()
        self.stable_since = None
        self.blocked_since = None
        # Start the second-window request as soon as the first window has
        # been crossed. It remains active while the aircraft flies through
        # the delivery circuit, so recognition never creates a stop at the
        # return window.
        if step["name"] == "window_1_cross":
            self.detect_window("window_2")
        if self.index < len(self.route):
            self.event(f"执行 {self.route[self.index]['name']}", step=self.index)

    def obstacle(self, vx, vy):
        speed = math.hypot(vx, vy)
        if not self.config["lidar_enabled"] or speed < .015 or not len(self.points):
            return False
        dx, dy = vx/speed, vy/speed
        p = self.points
        margin = self.config["obstacle_margin"]
        # Swept footprint along the command; ignore floor / overhead frame
        # only when they actually clear the complete skid-to-rotor envelope.
        along = p[:, 0]*dx + p[:, 1]*dy
        across = abs(-p[:, 0]*dy + p[:, 1]*dx)
        radius = .35*(abs(dx)+abs(dy))+margin
        lookahead = min(.90, .25 + speed*.05)
        near = ((along > 0) & (along < radius+.10+lookahead) &
                (across < radius) & (p[:, 2] > -.20-margin) & (p[:, 2] < .34+margin))
        return int(near.sum()) >= 2

    def tick(self):
        if self.finished:
            self.hold()
            return
        wall, now = time.monotonic(), self.now()
        fresh = (self.pose is not None and wall-self.pose_wall < self.config["sensor_timeout"] and
                 wall-self.package_wall < max(5., self.config["sensor_timeout"]))
        if self.config["lidar_enabled"]:
            fresh = fresh and wall-self.lidar_wall < self.config["sensor_timeout"]
        if not fresh:
            self.hold()
            if self.started or wall-self.created_wall > 90.:
                self.finish(False, "位姿、雷达或物资状态中断/启动超时，任务已停止")
            return
        p, heading = self.pose.position, yaw(self.pose.orientation)
        if not self.started:
            if not self.throw_ready:
                self.hold()
                if wall-self.created_wall > 90.:
                    self.finish(False, "投掷扩展启动超时")
                return
            hx, hy = self.config["home"]
            if math.hypot(p.x-hx, p.y-hy) > .15 or p.z > .3 or self.carried != set(self.order):
                self.finish(False, "需要全新仿真：无人机应在起降点且携带全部四种物资，勿同时运行最小示例")
                return
            self.started = True
            self.report["result"] = "running"
            self.report["started_sim"] = now
            self.step_started = now
            self.event("任务开始：识别穿窗 → 空中投掷 → 第三点远距识别 → 第四点降高投掷 → 直接穿窗返航")
        if now-self.last_trace >= .5:
            self.report["trajectory"].append([round(now, 3), p.x, p.y, p.z, heading, self.index])
            self.last_trace = now
            stamp = self.get_clock().now().to_msg()
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp, pose.pose = "odom", stamp, self.pose
            self.flight_path.poses.append(pose)
            self.flight_path.header.stamp = stamp
            self.path_pub.publish(self.flight_path)
            plan = FlightPath()
            plan.header = pose.header
            for move in self.route:
                if move["kind"] == "move":
                    waypoint = PoseStamped()
                    waypoint.header = pose.header
                    waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z = move["target"][:3]
                    waypoint.pose.orientation.w = 1.
                    plan.poses.append(waypoint)
            self.plan_pub.publish(plan)
            name = self.route[self.index]["name"] if self.index < len(self.route) else "COMPLETED"
            self.publish_label(f"{self.index+1}/{len(self.route)} {name}\nDropped: {len(self.dropped)}/4")
        self.monitor_throws()
        if self.finished:
            return
        if self.request and self.request["window"] == "window_2":
            self.detect_window("window_2")
            if now-self.request["started"] > self.config["detection_timeout"] and "window_2" not in self.report["detections"]:
                self.finish(False, "飞行中第二窗口识别超时")
                return
        if self.index >= len(self.route):
            if self.dropped != set(self.order) or len(self.report["drops"]) != 4 or any(
                    "landed" not in self.report["throws"].get(token, {}) for token in self.pending_throws):
                self.finish(False, "路线结束但物资状态不完整")
            elif set(self.report["detections"]) != {"window_1", "window_2"}:
                self.finish(False, "路线结束但双窗口识别不完整")
            else:
                self.finish(True, "完整 demo 完成：双窗口识别及穿越、四点投放确认、返航降落")
            return
        if now < self.departure_until:
            # Keep translating along the throw tangent while the cube clears
            # the tray. No zero command, acknowledgement wait, or yaw change.
            command = Twist()
            command.linear.x, command.linear.y = body_velocity(*self.departure_velocity, heading)
            if self.obstacle(command.linear.x, command.linear.y):
                self.finish(False, "抛掷离开段遇到障碍")
                return
            self.publish_velocity(command)
            return
        step = self.route[self.index]
        limit = self.config["detection_timeout"] if step["kind"] == "detect" else self.config["waypoint_timeout"]
        if now-self.step_started > limit:
            self.finish(False, f"{step['name']} 超时，已停止；请检查报告和相机图像")
            return
        if step["kind"] == "move":
            x, y, z, target_yaw = step["target"]
            ex, ey, ez, ea = x-p.x, y-p.y, z-p.z, angle(target_yaw-heading)
            distance = math.hypot(ex, ey)
            if step["name"] == "drop_4_approach" and distance > .55:
                ez = self.config["throw_z"] - p.z
            landed = step["name"] == "landing"
            precise = "drop_" in step["name"]
            flyby = step.get("flyby", False)
            if step["name"] == "drop_3_approach" and abs(ea) < 1.2:
                self.detect_window("window_2")
            tangent = step.get("tangent", (0., 0.))
            along = ex*tangent[0] + ey*tangent[1]
            across = abs(ex*tangent[1] - ey*tangent[0])
            if flyby and along < .04 and across < .12:
                if abs(ez) > .065 or abs(ea) > .12:
                    self.finish(False, f"{step['name']} 到达释放区但高度/航向未就绪")
                    return
                self.advance()
                self.trigger_throw(self.route[self.index])
                if not self.finished:
                    self.advance()
                    self.tick()  # Publish the next leg's velocity in this same tick.
                return
            tolerance = .012 if precise or landed else .025
            arrived = distance < tolerance and abs(ez) < .015 and abs(ea) < .035
            command = Twist()
            if arrived and not landed:
                self.advance()
                self.tick()
                return
            if arrived:
                self.stable_since = now if self.stable_since is None else self.stable_since
                self.hold()
                if now-self.stable_since > (1.0 if landed else .18):
                    self.advance()
                return
            self.stable_since = None
            velocity = step["speed"] if flyby else min(step["speed"], 6.*distance)
            vx, vy = body_velocity(ex*velocity/max(distance, 1e-9), ey*velocity/max(distance, 1e-9), heading)
            # Rotate during the third approach; it is a clear, known route.
            if abs(ea) > .18 and step["name"] != "drop_3_approach":
                vx, vy = 0., 0.
            if self.obstacle(vx, vy):
                vx, vy = 0., 0.
                if self.blocked_since is None:
                    self.blocked_since = now
                    self.event(f"{step['name']}：雷达检测到航线障碍，悬停等待")
                elif now-self.blocked_since > 10.:
                    self.finish(False, f"{step['name']} 航线持续受阻，已悬停")
                    return
            else:
                self.blocked_since = None
            command.linear.x, command.linear.y = vx, vy
            command.linear.z = clamp(3.5*ez, .65)
            # Complete the half-turn while translating toward release 3.
            command.angular.z = clamp(3.5*ea, 1.8)
            self.publish_velocity(command)
            if step["name"] == "return_home" and not self.crossed_second and p.x > self.config["windows"][1][0] + .36:
                self.crossed_second = True
                self.event("完成 window_2_cross", nonstop=True)
        elif step["kind"] == "detect":
            if self.detect_window(step["name"]):
                self.advance()
                self.tick()
            else:
                self.hold()
        else:
            self.trigger_throw(step)
            if not self.finished:
                self.advance()

    def trigger_throw(self, step):
        color = step["color"]
        if color not in self.carried:
            self.finish(False, f"{color} 不在携带列表中，不能确认本次投放")
            return
        now, p = self.now(), self.pose.position
        token = f"{self.session}_{self.index}_throw"
        command = f"{token} {color} {step['point'][0]:.8f} {step['point'][1]:.8f}"
        self.pending_throws[token] = dict(point=step["name"], color=color,
            sim_time=now, aircraft=[p.x,p.y,p.z], target=step["point"],
            trigger_velocity=self.measured_velocity[:], command=command,
            last_sent=now, attempts=1, recorded=False, motion_saved=False)
        speed = math.hypot(*self.measured_velocity[:2])
        if speed < .1:
            self.finish(False, "飞行速度不足，取消本次运动中投掷")
            return
        tangent = self.route[self.index-1]["tangent"]
        self.departure_velocity = tuple(v*self.route[self.index-1]["speed"] for v in tangent)
        # Keep moving along the release tangent for tray/skid clearance.
        # This is continuous flight, not a hover or confirmation wait.
        self.departure_until = now + .24
        self.drop.publish(String(data=command))
        self.event(f"{step['name']}：飞行中触发 {color} 投掷", token=token,
                   velocity=self.measured_velocity[:])

    def monitor_throws(self):
        now = self.now()
        for token, pending in self.pending_throws.items():
            if not pending["motion_saved"] and now-pending["sim_time"] > .8:
                self.report["throws"].setdefault(token, {})["aircraft_motion"] = [
                    sample for sample in self.motion_samples
                    if pending["sim_time"]-.2 <= sample[0] <= pending["sim_time"]+.8]
                pending["motion_saved"] = True
                self.save()
            if token in self.throw_results and pending["color"] in self.dropped:
                if not pending["recorded"]:
                    self.report["drops"].append({k: pending[k] for k in
                        ("point", "color", "sim_time", "aircraft", "target", "trigger_velocity")})
                    self.report["drops"][-1].update(confirmed=True, throw=self.throw_results[token])
                    pending["recorded"] = True
                    self.event(f"{pending['point']}：已异步确认投掷，航线继续")
            elif now-pending["last_sent"] > 2.:
                if pending["attempts"] >= 3:
                    self.finish(False, f"{pending['color']} 投掷未得到装置确认")
                    return
                self.drop.publish(String(data=pending["command"]))
                pending["attempts"] += 1
                pending["last_sent"] = now

    def detect_window(self, name):
        if name in self.report["detections"]:
            return True
        if self.request is None or self.request["window"] != name:
            self.detection = None
            self.last_request = -math.inf
            self.request = dict(started=self.now(), window=name, token=f"{self.session}_{self.index}",
                                evidence_file=str(self.output / f"{name}.jpg"))
        if self.now()-self.last_request > 1.:
            self.detect_request.publish(String(data=json.dumps(self.request)))
            self.last_request = self.now()
        if (self.detection and self.detection.get("evidence_file") == self.request["evidence_file"]
                and Path(self.request["evidence_file"]).is_file()):
            self.report["detections"][name] = self.detection
            self.event(f"{name} 相机确认：{self.detection['label']}", aircraft=[
                self.pose.position.x, self.pose.position.y, self.pose.position.z])
            self.detect_request.publish(String(data="cancel"))
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryMission()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.finish(False, "用户中断，已停止")
    except Exception as error:
        node.finish(False, f"任务异常：{error}")
        raise
    finally:
        if rclpy.ok():
            node.hold()
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
