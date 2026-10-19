# iCAN 任务二物资投送仿真用户练习工作区

新增完整任务 Demo（双窗口识别与穿越、四点投放、返航降落，不含裁判）：

```bash
bash scripts/run_full_demo.sh
```

详细操作、顺序参数、验证方法及适用范围见 [完整Demo.md](完整Demo.md)。以下保留官方原始练习版说明。

这个工作区面向参赛者和用户，提供 iCAN 任务二“城市应急物资投送”的基础仿真环境、UAV 传感器、飞控桥接和最小投放示例。

当前用户练习版不包含裁判系统、完整任务主流程和 Ark 视觉识别节点。用户可以基于公开接口实现自己的任务策略。

本目录是从原始最简工作空间复制出的独立 demo 版本。场景地面和两个窗户目标均使用 Gazebo 材质贴图；第一窗户默认使用 ArUco 二维码候选图，第二窗户默认使用老鹰/无人机候选图。

## 已提供内容

| 内容 | 说明 |
| --- | --- |
| `packages/ican_delivery_env_binary.tar.gz` | 预编译基础环境包，包含 Gazebo 场景、UAV 传感器、投放插件和飞控桥接。 |
| `src/ican_delivery_examples` | 用户示例源码包，演示如何订阅飞控当前位姿、发布飞控速度命令，并向 `/delivery/drop_cmd` 投放一个物块。 |
| `scripts/randomize_demo_world_textures.py` | 启动 Gazebo 前随机更换地面（可选）、第一窗户二维码和第二窗户贴图。 |

## 快速开始

```bash
cd <ican_delivery_user_ws>
bash scripts/00_install_ros2_gazebo.sh
bash scripts/01_setup_delivery_user_workspace.sh
source scripts/setup_env.sh
bash scripts/check_sim_env.sh
bash scripts/run_delivery_env.sh
```

`run_delivery_env.sh` 会打开三个终端标签：

- `delivery sim`：Gazebo 仿真场景、UAV、相机、雷达和投放插件
- `flight control`：飞控桥接包
- `minimal example`：自动运行最小投放示例

## 随机更换世界贴图

默认贴图已经随工作空间提供，直接启动仿真即可，不需要先运行随机脚本。启动文件会把当前 `ican_delivery_sim` 的 `share` 目录加入 `GAZEBO_MODEL_PATH`，世界文件使用 `model://ican_delivery_sim/...` 资源路径，因此工作空间解压到 `/home/cpm` 等其他目录也可以正常加载贴图。

如果需要随机更换贴图，请在启动仿真前运行脚本；脚本只负责选择并写入贴图材质：

```bash
cd <ican_delivery_demo_ws>
python3 scripts/randomize_demo_world_textures.py
bash scripts/run_delivery_env.sh
```

脚本每次会随机选择第一窗户的 ArUco 二维码和第二窗户的候选贴图。需要复现同一组选择时指定种子：

```bash
python3 scripts/randomize_demo_world_textures.py --seed 2026
```

只查看选择结果、不写文件：

```bash
python3 scripts/randomize_demo_world_textures.py --seed 2026 --dry-run
```

如果还要随机选择地面环境贴图，可提供一个包含图片的目录：

```bash
python3 scripts/randomize_demo_world_textures.py --ground-dir /path/to/ground_textures
```

本次选择会记录在 `binary_install/share/ican_delivery_sim/demo_texture_assignment.json`。修改贴图后需要重新启动 Gazebo 才会生效。

## 运行最小投放示例

启动脚本已经会自动运行一次示例。也可以另开终端手动运行：

```bash
cd <ican_delivery_user_ws>
source scripts/setup_env.sh
ros2 run ican_delivery_examples minimal_drop_demo
```

模板会让 UAV 原地起飞、下降一点、投放一个物块、再拉升并降落。默认投放 `red`，可改参数：

```bash
ros2 run ican_delivery_examples minimal_drop_demo --ros-args -p drop_color:=white
```

更多接口见 [仿真环境与接口说明.md](docs/仿真环境与接口说明.md)。
