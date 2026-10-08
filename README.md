# AUBO 视觉抓取工作空间

ROS 2 Humble 工作空间，默认模型为 `aubo_i10`。保留 Gazebo 仿真、MoveIt 规划、
RGB-D 视觉抓取、真实 AUBO 驱动及全部机器人模型。统一入口为 `./aubo.sh`。

## 常用命令

在工作空间根目录执行：

```bash
./aubo.sh build                 # 完整构建所有包，包括真实驱动和 SDK
./aubo.sh start gui             # 启动 Gazebo、工作站、控制器、MoveIt 和相机 TF
./aubo.sh pick                  # 默认生成 6 个方块并运行动态抓取队列
./aubo.sh pick 4                # 指定 1～6 个方块
./aubo.sh stop                  # 关闭工作站进程
```

无界面运行使用 `./aubo.sh start headless`；需要 Xvfb，可通过 `XVFB_BIN` 指定可执行文件。
仅生成物体使用 `./aubo.sh spawn 1`。GUI 和 headless 选择一种启动即可。

首次配置 RGB-D 网络缓冲区：

```bash
sudo ./aubo.sh setup-network
```

`build` 后的参数原样传给 colcon，例如 `./aubo.sh build --packages-select aubo_vision_grasp`。
默认完整构建；不区分或裁减仿真与真实驱动包。开发环境可在干净的构建/安装目录使用
`--symlink-install`；现有目录混用复制安装与链接安装时，应先备份再重建。

## 真实机械臂与 ROS 工具

`ros` 子命令加载 ROS、工作空间和公共配置，然后原样调用 ROS CLI：

```bash
./aubo.sh ros launch aubo_ros2_driver aubo_control.launch.py --show-args
./aubo.sh ros control list_controllers
./aubo.sh ros node list --no-daemon
```

真实机械臂连接参数（如 `robot_ip`、`aubo_type`）继续由
`aubo_ros2_driver/launch/aubo_control.launch.py` 管理。详细驱动使用说明见
[src/aubo_ros2_driver/README.md](src/aubo_ros2_driver/README.md)。
本次结构整理不改变该包的硬件控制逻辑；构建驱动 SDK 时可能需要访问其下载地址。
注意：实机 launch 保留原有默认型号 `aubo_i5`，连接 i10 时须显式传入
`aubo_type:=aubo_i10` 和实际 `robot_ip`；`ROBOT_MODEL` 是工作站启动接口，
不会改写 `ros` 子命令转发的实机参数。

## 文件职责

```text
aubo.sh                          唯一公共命令入口
scripts/
  lib/env.sh                     工作空间定位、ROS 环境与配置加载
  start.sh                       仿真工作站启动和就绪检查
  stop.sh                        进程树清理
  pick.sh                        抓取前检查、生成物体、运行视觉队列
  spawn_cubes.py                 统一的 1～6 物体生成器
  setup_network.sh               一次性网络缓冲区设置
src/
  aubo_ros2_driver/               模型、Gazebo、MoveIt、真实驱动及消息包
  aubo_workcell_description/      工作站/相机模型及共享配置
  aubo_vision_grasp/              检测、目标队列、运动规划与测试
  suction_gripper_gazebo/         吸盘 Gazebo 插件
models/grasp_cube/               抓取物体模型
docs/                            模型接口与开发约定
tests/                           工作空间命令入口测试
archive/                         本地可恢复归档，不进入 Git
build/ install/ log/ run_logs/    构建产物与运行日志，不进入 Git
```

启动链路：`aubo.sh start` → Gazebo → 工作站/相机 → 控制器 → MoveIt → 碰撞场景/相机 TF。
抓取链路：`aubo.sh pick` → 物体生成 → `visual_pick_system.launch.py` →
检测器与队列 → 运动规划/执行及吸盘。检测器和队列由同一个 launch 管理生命周期。

## 配置与开发

统一参数文件：[workcell_layout.env](src/aubo_workcell_description/config/workcell_layout.env)。
脚本经 `scripts/lib/env.sh` 加载并导出；新增参数放到该文件及 Python 的
`system_config.py`，不要再在入口脚本复制默认值。

可用 `AUBO_CONFIG_FILE` 指向自定义配置，`AUBO_GRASP_WS` 指向另一工作空间。
默认根据脚本位置定位工作空间，不依赖当前目录。模型切换接口见
[docs/robot_model_interface.md](docs/robot_model_interface.md)。

```bash
ROBOT_MODEL=aubo_i10 ./aubo.sh start gui
./aubo.sh ros run aubo_vision_grasp red_target_detector
```

程序日志保存在 `run_logs/`；构建日志由 colcon 写入 `log/`。
排查时优先检查 `gazebo.log`、`moveit.log`、`visual_pick_system.log`。

## 验证与 Git

```bash
python3 -m pytest -q tests
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/aubo_vision_grasp/test/test_system_config.py \
  src/aubo_vision_grasp/test/test_target_queue.py \
  src/aubo_vision_grasp/test/test_receiving_bin_layout.py
```

[.gitignore](.gitignore) 忽略构建/安装目录、日志、Python 缓存、VS Code 索引和本地归档；
保留代码、真实驱动、模型网格和项目默认配置。`aubo_description/xacro.sh` 自动生成的
直接位于 `urdf/` 下的 `*.urdf.xacro` 也被忽略，其来源 `*.urdf` 和
`urdf/xacro/` 中手写 Xacro 继续提交。

确认本地内容后可自行初始化仓库：

```bash
git init
git add .
git status --short
git commit -m "Initial workspace"
```

`.gitignore` 只控制版本跟踪，不会删除现有文件或释放磁盘空间。
构建前应保留源码；运行时需要 `install/` 及它引用的源文件/构建产物。

## 旧入口迁移

| 原入口 | 新命令 |
| --- | --- |
| `start_gui_aubo.sh` | `./aubo.sh start gui` |
| `start_headless_aubo.sh` | `./aubo.sh start headless` |
| `start_aubo_system.sh` | `./aubo.sh start gui` 或 `headless` |
| `cleanup_aubo_system.sh` | `./aubo.sh stop`，保留 `--force`/`--legacy` |
| `run_random_pick.sh`、`quick_pick.sh` | `./aubo.sh pick [数量]` |
| 单物体/四物体/多物体生成脚本 | `./aubo.sh spawn 1`、`spawn 4`、`spawn 6` |
| `install_aubo_socket_buffers.sh` | `sudo ./aubo.sh setup-network` |
| `load_aubo_config.sh` | `source scripts/lib/env.sh; aubo_load_config` |

生成器统一采用当前随机投放逻辑；旧四物体脚本的固定间距模式保存在归档中。
旧监督脚本由现有 ROS launch 生命周期管理取代。
旧入口和旧 README 保存在 `archive/before-layout-refactor-20261008.tar.gz`；
历史源码快照保存在 `archive/source-snapshots-20261008.tar.gz`，对应清单同目录。
这两个归档不进入 Git，需要保留历史时应另外备份。
