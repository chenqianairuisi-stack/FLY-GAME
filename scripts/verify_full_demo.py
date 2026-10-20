#!/usr/bin/env python3
"""Read-only demo self-check, not a referee or competition score system.

The controller never uses Gazebo model states. This separate tool observes
physical landing positions after a run to validate the implementation.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ModelStates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="本次 demo 的 report.json")
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    assert report["result"] == "completed", report.get("reason", "任务未完成")
    assert set(report["detections"]) == {"window_1", "window_2"}
    assert all(d["confirmed_frames"] >= 3 for d in report["detections"].values())
    assert [d["color"] for d in report["drops"]] == report["order"]
    assert len(report.get("throws", {})) == 4, "缺少四次投掷轨迹记录"
    names = [e["message"] for e in report["events"]]
    assert "完成 window_1_cross" in names and "完成 window_2_cross" in names
    assert "完成 landing" in names
    rclpy.init()
    node = rclpy.create_node("delivery_read_only_verifier")
    states = []
    subscription = node.create_subscription(ModelStates, "/gazebo/model_states", states.append, 1)
    deadline = time.monotonic()+15
    while not states and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    assert states, "未收到 Gazebo 状态，请在仿真保持开启时运行"
    models = dict(zip(states[-1].name, states[-1].pose))
    evidence = dict(detections={k: v["label"] for k, v in report["detections"].items()}, drops=[], throws=[])
    for item in report["throws"].values():
        release, landed = item.get("released"), item.get("landed")
        assert release and landed, f"{item.get('color', 'unknown')} 缺少释放或轨迹记录"
        motion = item.get("aircraft_motion", [])
        assert len(motion) >= 8, "缺少投掷前后的无人机运动记录"
        trigger = next(d["sim_time"] for d in report["drops"] if d["color"] == release["color"])
        # The fast return can reach home within the old 0.8 s observation
        # interval. Check continuous flight around release, not home braking.
        throwing = [p for p in motion if trigger-.1 <= p[0] <= trigger+.25]
        assert len(throwing) >= 8, "释放区间运动采样不足"
        speeds = [math.hypot(*sample[4:6]) for sample in throwing]
        assert min(speeds) > .20, f"{release['color']} 投掷期间存在停顿：{min(speeds):.3f} m/s"
        travelled = sum(math.dist(a[1:3], b[1:3]) for a,b in zip(throwing, throwing[1:]))
        duration = throwing[-1][0]-throwing[0][0]
        # Require continuous travel at >0.2 m/s over the sampled interval;
        # a fixed 0.2 m displacement would wrongly reject 0.5 m/s flight.
        assert duration > 0 and travelled > .20*duration, "投掷期间无人机没有持续前进"
        samples = landed.get("samples", [])
        assert len(samples) > 10, "投掷轨迹采样不足"
        airborne = [p for p in samples if p[3] > .08]
        assert len(airborne) >= 8, "空中轨迹采样不足"
        origin, velocity = release["position"], release["velocity"]
        residuals = []
        for t, x, y, z in airborne:
            predicted = [origin[0]+velocity[0]*t, origin[1]+velocity[1]*t,
                         origin[2]+velocity[2]*t-.5*9.8*t*t]
            residuals.append(math.dist([x,y,z], predicted))
        assert max(residuals) < .008, f"{release['color']} 轨迹偏离自由抛物线：{max(residuals):.4f} m"
        t,x,y,z = airborne[-1]
        target = release["target"]
        end = samples[-1]
        assert math.hypot(end[1]-target[0], end[2]-target[1]) < .075, f"{release['color']} 投掷落点超出内环"
        assert math.hypot(x-release["position"][0], y-release["position"][1]) > .10, "投掷没有水平位移"
        evidence["throws"].append(dict(color=release["color"], ballistic_sample=[t,x,y,z],
            target=target, landing_xy=end[1:3], min_aircraft_speed_mps=min(speeds),
            max_ballistic_error_m=max(residuals)))
    for drop in report["drops"]:
        pose = models[f"package_{drop['color']}"].position
        x, y = drop["target"]
        error = math.hypot(pose.x-x, pose.y-y)
        evidence["drops"].append(dict(color=drop["color"], target=[x, y],
            actual=[pose.x, pose.y, pose.z], error_m=error))
        assert error < .075, f"{drop['color']} 未落在内环：{error:.3f} m"
        assert pose.z < .08, f"{drop['color']} 尚未落地"
    home = report["config"]["home"]
    aircraft = models["delivery_uav"].position
    evidence["landing_error_m"] = math.hypot(aircraft.x-home[0], aircraft.y-home[1])
    assert evidence["landing_error_m"] < .03 and abs(aircraft.z-.2) < .02
    evidence["elapsed_sim_sec"] = report["elapsed_sim_sec"]
    evidence["peak_speed_mps"] = report.get("peak_speed_mps")
    if "max_speed" in report["config"]:
        limit = report["config"]["max_speed"]
        peak = report.get("peak_3d_speed_mps")
        assert peak is not None and math.isfinite(peak) and 0 < peak <= limit+1e-6, f"实际三维速度超限：{peak} m/s，限制 {limit} m/s"
        evidence["max_speed_mps"] = limit
        evidence["peak_3d_speed_mps"] = peak
    evidence["passed"] = True
    args.report.with_name("verification.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2))
    print(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
