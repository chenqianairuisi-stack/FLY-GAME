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
    evidence = dict(detections={k: v["label"] for k, v in report["detections"].items()}, drops=[])
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
    evidence["passed"] = True
    args.report.with_name("verification.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2))
    print(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
