# AUBO i10 随机物料视觉抓取工作站仿真 README

## 1. 当前系统

工作空间：

```bash
~/aubo_grasp_ws
```

主要包：

```text
AUBO i10              aubo_ros2_driver
Gazebo                 aubo_gazebo
MoveIt2                aubo_moveit_config
吸盘插件               suction_gripper_gazebo
工作站                 aubo_workcell_description
视觉与抓取程序         aubo_vision_grasp
```

当前正式流程：

```text
随机生成 4 个红色方块
        ↓
固定 RGB-D 相机检测
        ↓
计算目标在 base_link 下的 XYZ
        ↓
AUBO 自动抓取
        ↓
放入左侧小箱
        ↓
重新检测剩余物体
        ↓
4 → 3 → 2 → 1 → 0
        ↓
平台为空后自动停止
```

---

## 2. 完整重启

```bash
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
pkill -9 move_group 2>/dev/null || true
sleep 2
```

检查：

```bash
ps aux | grep -E 'gzserver|gzclient|move_group|rviz2' | grep -v grep
```

---

## 3. 终端 1：启动 AUBO i10 + 吸盘

```bash
cd ~/aubo_grasp_ws

conda deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

export GAZEBO_MODEL_DATABASE_URI=""

export GAZEBO_MODEL_PATH="$HOME/aubo_grasp_ws/install/aubo_workcell_description/share/aubo_workcell_description/models:$HOME/aubo_grasp_ws/install/aubo_description/share:/opt/ros/humble/share${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

export GAZEBO_PLUGIN_PATH="$HOME/aubo_grasp_ws/install/suction_gripper_gazebo/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

ros2 launch aubo_gazebo aubo_gazebo.launch.py   robot_type:=aubo_i10   launch_rviz:=false
```

保持运行。

---

## 4. 终端 2：生成工作站

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

export GAZEBO_MODEL_PATH="$HOME/aubo_grasp_ws/install/aubo_workcell_description/share/aubo_workcell_description/models:$HOME/aubo_grasp_ws/install/aubo_description/share:/opt/ros/humble/share${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

ros2 launch aubo_workcell_description spawn_workcell.launch.py
```

Gazebo 中应出现：

```text
AUBO i10
原吸盘
大白色取料平台
左侧接料小箱
相机支架
固定 RGB-D 相机
```

---

## 5. 终端 3：确认轨迹控制器

```bash
source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

ros2 control list_controllers
```

正常：

```text
joint_state_broadcaster        active
joint_trajectory_controller    active
```

如果未激活：

```bash
ros2 control load_controller   --set-state active   joint_trajectory_controller
```

检查吸盘服务：

```bash
ros2 service list | grep suction
```

正常：

```text
/suction_on
/suction_off
```

---

## 6. 终端 4：启动 MoveIt2

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

ros2 launch aubo_moveit_config aubo_moveit.launch.py   aubo_type:=aubo_i10
```

保持运行。

---

## 7. 终端 5：加载工作站 Planning Scene

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

python3 ~/aubo_grasp_ws/src/aubo_vision_grasp/aubo_vision_grasp/setup_workcell_scene.py --ros-args -p use_sim_time:=true
```

检查：

```bash
ros2 service list | grep compute_cartesian_path
ros2 action list | grep -E 'move_action|execute_trajectory'
```

正常：

```text
/compute_cartesian_path
/move_action
/execute_trajectory
```

---

## 8. 终端 6：启动相机 TF

先检查是否已经存在：

```bash
ros2 node list | grep -E 'camera_mount_tf|camera_optical_tf'
```

如果没有：

```bash
source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

nohup ros2 run tf2_ros static_transform_publisher   --x 0.026   --y -0.914   --z 0.790   --roll 0   --pitch 1.5707963   --yaw 0   --frame-id base_link   --child-frame-id camera_mount   --ros-args   -r __node:=camera_mount_tf   >/tmp/camera_mount_tf.log 2>&1 &
```

然后：

```bash
nohup ros2 run tf2_ros static_transform_publisher   --x 0   --y 0   --z 0   --roll -1.5707963   --pitch 0   --yaw -1.5707963   --frame-id camera_mount   --child-frame-id camera_link   --ros-args   -r __node:=camera_optical_tf   >/tmp/camera_optical_tf.log 2>&1 &
```

检查：

```bash
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_link
```

当前平移应接近：

```text
x =  0.026
y = -0.914
z =  0.790
```

---

## 9. 检查 RGB-D 相机

当前实际话题：

```text
/camera/workcell_camera/image_raw
/camera/workcell_camera/camera_info
/camera/workcell_camera/depth/image_raw
/camera/workcell_camera/depth/camera_info
/camera/workcell_camera/points
```

检查深度：

```bash
ros2 topic echo --once /camera/workcell_camera/depth/image_raw | grep -E 'height:|width:|encoding:'
```

已验证：

```text
height: 480
width: 640
encoding: 32FC1
```

检查点云：

```bash
ros2 topic echo --once /camera/workcell_camera/points | grep -E 'frame_id|height:|width:'
```

已验证：

```text
frame_id: camera_link
height: 480
width: 640
```

可视化：

```bash
ros2 run rqt_image_view rqt_image_view
```

RGB：

```text
/camera/workcell_camera/image_raw
```

Depth：

```text
/camera/workcell_camera/depth/image_raw
```

---

## 10. 终端 7：启动视觉检测

该终端保持运行：

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/aubo_vision_grasp/aubo_vision_grasp/red_target_detector.py --ros-args -p use_sim_time:=true
```

输出：

```text
/vision/target_pose
/vision/target_count
```

其中：

```text
/vision/target_pose
```

为当前工作台目标在 `base_link` 下的位置。

```text
/vision/target_count
```

为工作台当前剩余目标数。

当前工作台 ROI：

```text
x: -0.30 ~ +0.30 m
y: -1.10 ~ -0.47 m
z: -0.02 ~ +0.10 m
```

小箱约：

```text
x ≈ -0.605 m
```

因此已放进小箱的红块会被过滤。

---

## 11. 终端 8：随机生成 4 个物料

当前正式版本：

```text
数量：固定 4 个
位置：随机
```

运行：

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 ~/aubo_grasp_ws/spawn_random_cubes.py
```

脚本应设置：

```python
N = 4
```

但每次位置随机。

---

## 12. 检查视觉结果

```bash
ros2 topic echo --once /vision/target_count
```

正常：

```text
data: 4
```

检查当前目标：

```bash
ros2 topic echo --once /vision/target_pose
```

例如：

```yaml
header:
  frame_id: base_link

pose:
  position:
    x: 0.08
    y: -0.67
    z: 0.034
```

---

## 13. 终端 9：自动抓完 4 个随机物料

```bash
cd ~/aubo_grasp_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/aubo_vision_grasp/aubo_vision_grasp/visual_pick_all.py --ros-args -p use_sim_time:=true
```

流程：

```text
target_count = 4
        ↓
读取新的 target_pose
        ↓
PICK_SAFE
        ↓
PICK_DESCEND
        ↓
suction_on
        ↓
PICK_LIFT
        ↓
TRANSFER
        ↓
PLACE_ENTRY
        ↓
PLACE_DESCEND
        ↓
suction_off
        ↓
PLACE_RETREAT_INNER
        ↓
PLACE_RETREAT_SAFE
        ↓
重新检测
        ↓
4 → 3 → 2 → 1 → 0
```

当：

```text
target_count = 0
```

程序应输出：

```text
SOURCE TABLE EMPTY
TOTAL PICKED = 4
AUTO PICK COMPLETED
```

并自动停止。

---

## 14. 当前抓取与放置参数

工作台表面附近：

```text
z ≈ -0.016 m
```

5 cm 方块顶部：

```text
z ≈ 0.034 m
```

抓取高度：

```text
contact_z = object_top_z + 0.020
```

典型：

```text
0.034 + 0.020 = 0.054 m
```

安全高度：

```text
safe_z = 0.300 m
```

接料箱：

```text
place_x = -0.605
```

当前可轮换的安全 y：

```text
T1 = -0.806
T2 = -0.866
T3 = -0.926
```

当前释放高度：

```text
place_release_z = -0.150
```

退出：

```text
PLACE_RETREAT_INNER
z = -0.050

PLACE_RETREAT_SAFE
z = +0.300
```

---

## 15. 重置物料

关闭吸盘：

```bash
ros2 service call /suction_off std_srvs/srv/Trigger "{}"
```

删除 4 个方块：

```bash
for i in 1 2 3 4; do
  ros2 service call     /delete_entity     gazebo_msgs/srv/DeleteEntity     "{name: 'grasp_cube_$i'}"     >/dev/null 2>&1 || true
done
```

删除旧单物体：

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'grasp_cube'}" >/dev/null 2>&1 || true
```

重新随机：

```bash
python3 ~/aubo_grasp_ws/spawn_random_cubes.py
```

---

## 16. 日常启动顺序

```text
1. Gazebo + AUBO i10 + suction
        ↓
2. spawn workcell
        ↓
3. 确认 joint_trajectory_controller active
        ↓
4. MoveIt2
        ↓
5. setup_workcell_scene.py
        ↓
6. camera TF
        ↓
7. red_target_detector.py
        ↓
8. spawn_random_cubes.py
        ↓
9. 确认 /vision/target_count = 4
        ↓
10. visual_pick_all.py
        ↓
11. 4 → 3 → 2 → 1 → 0
        ↓
12. SOURCE TABLE EMPTY
```

---

## 17. 旧固定四点流程

以下程序不再作为随机视觉抓取的主流程：

```text
four_corner_pick_place.py
```

旧固定点仅用于回归测试：

```text
P1 = (-0.25, -0.52)
P2 = (+0.25, -0.52)
P3 = (+0.25, -1.05)
P4 = (-0.25, -1.05)
```

---

## 18. 常用检查命令

控制器：

```bash
ros2 control list_controllers
```

MoveIt：

```bash
ros2 service list | grep compute_cartesian_path
ros2 action list | grep -E 'move_action|execute_trajectory'
```

吸盘：

```bash
ros2 service list | grep suction
```

相机：

```bash
ros2 topic list | grep '^/camera'
```

TF：

```bash
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_link
```

目标数量：

```bash
ros2 topic echo --once /vision/target_count
```

当前目标：

```bash
ros2 topic echo --once /vision/target_pose
```

关节：

```bash
ros2 topic echo --once /joint_states
```

---

## 19. 当前系统结构

```text
Gazebo
 ├── AUBO i10
 ├── suction gripper
 ├── workcell
 ├── source table
 ├── receiving bin
 ├── fixed RGB-D camera
 └── grasp_cube_1 ~ grasp_cube_4
                │
                ▼
RGB + Depth + CameraInfo
                │
                ▼
red_target_detector.py
                │
        ┌───────┴────────┐
        ▼                ▼
/vision/target_pose  /vision/target_count
        │                │
        └───────┬────────┘
                ▼
        visual_pick_all.py
                │
                ▼
      visual_pick_to_bin.py
                │
        ┌───────┴─────────┐
        ▼                 ▼
      MoveIt2          suction
        │                 │
        └───────┬─────────┘
                ▼
            AUBO i10
                │
                ▼
        receiving bin
```

---

## 20. 当前开发状态

已完成：

```text
AUBO i10 Gazebo 仿真                     ✓
吸盘 attach / detach                    ✓
MoveIt2 + Cartesian                     ✓
工作站 Planning Scene                  ✓
固定 Eye-to-Hand RGB-D                 ✓
RGB / Depth / CameraInfo               ✓
PointCloud2                            ✓
camera_link → base_link TF             ✓
单随机物体定位                          ✓
单随机物体自动抓放                      ✓
多红色目标检测                          ✓
工作台 ROI                             ✓
忽略小箱内已抓物体                      ✓
/vision/target_count                    ✓
固定数量 4 个、位置随机                  ✓
逐个重新感知                            ✓
自动循环抓取                            ✓
平台为空后自动停止                      ✓
```

当前正式测试目标：

```text
固定生成 4 个随机位置红色方块
        ↓
RGB-D 在线检测
        ↓
逐个抓取
        ↓
全部放入左侧小箱
        ↓
target_count = 0 后自动结束
```
