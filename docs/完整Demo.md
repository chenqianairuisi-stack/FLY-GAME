# 城市应急物资投送完整 Demo

此 Demo 按当前练习场景及已确认的路线实现：起飞 → 识别第一窗户 ArUco → 降低高度穿窗 → 四个固定点依次投放 → 识别第二窗户图片 → 降低高度穿窗 → 返回原起降点并降落。不包含裁判、计分或裁判事件声明。

工作区没有完整竞赛规则书，因此这里的完成条件是上述练习流程；识别结果不擅自解释成投放指令，物资顺序通过参数明确指定。拿到正式规则后，可进一步调整动作、限时和识别结果的含义。

## 一条命令运行

先关闭之前启动的 Gazebo、飞控和最小示例。进入实际工作区（包含 scripts、src 的那一层）：

```bash
cd /home/chen-qian/Desktop/demo_delivery_ws/demo_delivery_ws
bash scripts/run_full_demo.sh
```

首次运行会自动编译新增入口。此脚本不运行原来的最小投放示例。默认同时打开 Gazebo 和 RViz，任务会在位姿、雷达及全部物资就绪后自动开始。RViz 显示前视相机、识别标注、点云、碰撞箱、橙色计划路线、绿色实际轨迹和当前步骤。任务结束后保留场景用于查看四个物块的落点。

重复演示时按 Ctrl+C 关闭整套程序，再执行启动命令。物块已释放后，不能直接在同一场景重跑任务。

无界面运行（仍使用真实 Gazebo 相机）：

```bash
bash scripts/run_full_demo.sh gui:=false rviz:=false
```

启动脚本使用 `PYTHONNOUSERSITE=1`，避开用户目录 NumPy 2 与 Ubuntu OpenCV/cv_bridge 的 ABI 冲突，不修改全局 Python 包。

## 指定投放顺序

点位按 drop_1 → drop_2 → drop_3 → drop_4 固定访问。编号含义为 1=红、2=黄、3=黑、4=白；默认 `1234`。例如黑、红、白、黄：

```bash
bash scripts/run_full_demo.sh drop_order:=3142
```

也可使用 `drop_order:=black,red,white,yellow`。每种物资必须且只能出现一次。不根据 Gazebo 模型名里的旧颜色名称推断点位要求。

## 随机贴图

关闭仿真后，可先用官方脚本更换贴图，再运行完整 Demo：

```bash
python3 scripts/randomize_demo_world_textures.py --seed 2026
bash scripts/run_full_demo.sh
```

识别节点从 `/uav/camera/image` 实际检测：第一窗口使用 ArUco 4x4_50，第二窗口使用官方候选图库的 SIFT 特征匹配和 RANSAC 几何校验，连续三帧一致才确认。支持贴图镜像；检测结果和标注图会保存。识别过程不读取贴图随机结果 JSON 或材质配置。第二窗口的识别范围为随包候选图库，未知的新图片需要补充图库或另换识别模型。

## 路线与控制

| 项目 | 默认值 |
| --- | --- |
| 起降中心 | (3.07, -2.38) |
| 第一窗户中心线 | (1.925, -0.3275) |
| 第二窗户中心线 | (1.925, -2.3725) |
| 四个投放点 | (1.12, -0.15)、(0.02, -0.67)、(0.02, -2.03)、(1.12, -2.53) |
| 穿窗高度 / 速度 | 0.74 m / 0.16 m/s |
| 观察高度 / 投放高度 | 1.27 m / 0.55 m |
| 巡航速度 | 最高 0.28 m/s |

控制器订阅 `/flight_control/current_pose`，将世界坐标误差转换到机体坐标，通过 `/flight_control/cmd_vel` 控制；不发送瞬移命令。先在窗口前观察上方贴图，再下降并沿中心线穿窗，考虑 0.70 m 机体宽度、下部脚架和上部旋翼的余量。投放时补偿四格托盘各自的 4 cm 偏移，悬停后发布颜色，并等待 `/delivery/package_state` 确认释放。

这是已知固定场地的航点方案。雷达会检查前向扫掠空间，遇障碍悬停，持续受阻则失败退出；没有实现任意未知场景的自动绕障。位姿、雷达或物资状态中断，以及识别/航点超时，也会停止并保存原因。

可调整参数见 `src/ican_delivery_examples/config/full_demo.yaml`。修改 Python 源码后可直接重启（使用 symlink-install）；修改入口、launch 或资源安装配置后执行：

```bash
source scripts/setup_env.sh
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select ican_delivery_examples
```

## 查看结果和停止任务

每次运行保存到 `demo_runs/日期_时间/`：

- `report.json`：完成/失败状态、步骤、识别结果、投放确认、轨迹与配置。
- `window_1.jpg`、`window_2.jpg`：识别标注图。

终端出现“完整 demo 完成”且报告 `result` 为 `completed` 表示流程完成。失败退出码为 1。该结果只反映本 Demo 的流程，不代表官方裁判得分。

另开终端可监控或停止：

```bash
source scripts/setup_env.sh
ros2 topic echo /delivery/mission_status --qos-durability transient_local
ros2 topic pub --once /delivery/mission_command std_msgs/msg/String '{data: abort}'
```

停止命令发送零速度让无人机悬停；Ctrl+C 关闭整套仿真。

仿真保持开启时，可对已完成报告做独立的只读物理落点检查（不参与控制、不计分）：

```bash
source scripts/setup_env.sh
PYTHONNOUSERSITE=1 python3 scripts/verify_full_demo.py demo_runs/本次目录/report.json
```

工具检查四个物块实际落地且位于半径 7.5 cm 的内环、返回起降点误差小于 3 cm，并保存 `verification.json`。这是开发验证工具，不是裁判系统。

## 本机实测

已在官方 Gazebo 场景中跑通默认 `1234` 和自定义 `3142` 两种投放顺序。最终带 Gazebo、RViz 界面的默认顺序验证记录位于 [report.json](../demo_runs/validation/20261018_150452_791395/report.json)，物理检查结果位于 [verification.json](../demo_runs/validation/20261018_150452_791395/verification.json)。

- 全程耗时 77.9 秒仿真时间，完成两个窗口识别及穿越、四点投放、返航降落。
- 四个物块实际落点距目标中心均小于 0.9 mm，全部落在内环；返航落点误差约 0.27 mm。这些数值来自当前简化仿真物理，不代表实机精度。
- 两个识别结果均连续三帧确认，分别保存 [第一窗口截图](../demo_runs/validation/20261018_150452_791395/window_1.jpg) 和 [第二窗口截图](../demo_runs/validation/20261018_150452_791395/window_2.jpg)。
- 自动测试 16 项通过，覆盖路线净空、托盘补偿、顺序校验、候选图识别、ArUco 检测、障碍保护及识别证据保存。

测试命令：

```bash
source scripts/setup_env.sh
PYTHONNOUSERSITE=1 python3 -m pytest src/ican_delivery_examples/test -q
```
