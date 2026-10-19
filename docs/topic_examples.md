# 常用话题示例

查看 UAV 当前位姿：

```bash
ros2 topic echo /flight_control/current_pose
```

飞控速度控制：

```bash
ros2 topic pub /flight_control/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.12}, angular: {z: 0.0}}" -r 10
```

投放物块：

```bash
ros2 topic pub --once /delivery/drop_cmd std_msgs/msg/String "{data: red}"
ros2 topic pub --once /delivery/drop_cmd std_msgs/msg/String "{data: 1}"
```

查看物块状态：

```bash
ros2 topic echo /delivery/package_state
```
