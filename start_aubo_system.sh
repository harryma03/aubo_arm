#!/bin/bash

set -e

WS="$HOME/aubo_grasp_ws"
LOG="$WS/run_logs"

mkdir -p "$LOG"

echo "=========================================="
echo " AUBO i10 WORKCELL AUTO START"
echo "=========================================="

cd "$WS"

conda deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export GAZEBO_MODEL_DATABASE_URI=""

export GAZEBO_MODEL_PATH="$WS/install/aubo_workcell_description/share/aubo_workcell_description/models:$WS/install/aubo_description/share:/opt/ros/humble/share${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

export GAZEBO_PLUGIN_PATH="$WS/install/suction_gripper_gazebo/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"


# ============================================================
# 0. 清理旧进程
# ============================================================

echo
echo "[0/7] Cleaning old processes..."

pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
pkill -9 move_group 2>/dev/null || true

pkill -f red_target_detector.py 2>/dev/null || true
pkill -f camera_mount_tf 2>/dev/null || true
pkill -f camera_optical_tf 2>/dev/null || true

sleep 2


# ============================================================
# 1. Gazebo + AUBO + suction
# ============================================================

echo
echo "[1/7] Starting Gazebo + AUBO i10 + suction..."

nohup ros2 launch \
  aubo_gazebo \
  aubo_gazebo.launch.py \
  robot_type:=aubo_i10 \
  launch_rviz:=false \
  > "$LOG/gazebo.log" 2>&1 &

echo "Waiting for Gazebo..."

for i in $(seq 1 60); do

    if ros2 service list 2>/dev/null \
        | grep -q '^/spawn_entity$'; then

        echo "Gazebo ready."
        break
    fi

    sleep 1

    if [ "$i" -eq 60 ]; then
        echo "ERROR: Gazebo startup timeout."
        echo "Check:"
        echo "  tail -100 $LOG/gazebo.log"
        exit 1
    fi
done


# ============================================================
# 2. Workcell
# ============================================================

echo
echo "[2/7] Spawning workcell..."

nohup ros2 launch \
  aubo_workcell_description \
  spawn_workcell.launch.py \
  > "$LOG/workcell.log" 2>&1 &

echo "Waiting for RGB-D camera..."

for i in $(seq 1 30); do

    if ros2 topic list 2>/dev/null \
        | grep -q '^/camera/workcell_camera/depth/image_raw$'; then

        echo "Workcell + RGB-D camera ready."
        break
    fi

    sleep 1

    if [ "$i" -eq 30 ]; then
        echo "WARNING: Camera topic not detected yet."
        echo "Check:"
        echo "  tail -100 $LOG/workcell.log"
    fi
done


# ============================================================
# 3. Controller
# ============================================================

echo
echo "[3/7] Checking trajectory controller..."

for i in $(seq 1 30); do

    if ros2 control list_controllers 2>/dev/null \
        | grep -q joint_state_broadcaster; then
        break
    fi

    sleep 1
done

if ros2 control list_controllers 2>/dev/null \
    | grep joint_trajectory_controller \
    | grep -q active; then

    echo "joint_trajectory_controller already active."

else

    echo "Activating joint_trajectory_controller..."

    ros2 control load_controller \
      --set-state active \
      joint_trajectory_controller
fi


# ============================================================
# 4. MoveIt
# ============================================================

echo
echo "[4/7] Starting MoveIt2..."

nohup ros2 launch \
  aubo_moveit_config \
  aubo_moveit.launch.py \
  aubo_type:=aubo_i10 \
  > "$LOG/moveit.log" 2>&1 &

echo "Waiting for MoveIt..."

for i in $(seq 1 60); do

    SERVICE_OK=0
    ACTION_OK=0

    ros2 service list 2>/dev/null \
        | grep -q '^/compute_cartesian_path$' \
        && SERVICE_OK=1

    ros2 action list 2>/dev/null \
        | grep -q '^/execute_trajectory$' \
        && ACTION_OK=1

    if [ "$SERVICE_OK" -eq 1 ] \
        && [ "$ACTION_OK" -eq 1 ]; then

        echo "MoveIt ready."
        break
    fi

    sleep 1

    if [ "$i" -eq 60 ]; then
        echo "ERROR: MoveIt startup timeout."
        echo "Check:"
        echo "  tail -100 $LOG/moveit.log"
        exit 1
    fi
done


# ============================================================
# 5. Planning Scene
# ============================================================

echo
echo "[5/7] Waiting for Planning Scene..."

# MoveIt services may appear before PlanningSceneMonitor is fully ready.
sleep 5

SCENE_SCRIPT="$WS/src/aubo_vision_grasp/aubo_vision_grasp/setup_workcell_scene.py"

SCENE_OK=0

for attempt in 1 2 3; do

    echo "Applying Planning Scene... attempt $attempt/3"

    python3 \
      "$SCENE_SCRIPT" \
      --ros-args \
      -p use_sim_time:=true

    sleep 2

    OBJECTS=$(
      ros2 service call \
        /get_planning_scene \
        moveit_msgs/srv/GetPlanningScene \
        "{components: {components: 24}}" \
        2>/dev/null \
        | grep -o "id='[^']*'" | wc -l || true
    )

    if [ "$OBJECTS" -ge 5 ]; then
        echo "Planning Scene ready: $OBJECTS collision objects found."
        SCENE_OK=1
        break
    fi

    echo "Planning Scene not ready, retrying..."
    sleep 2
done

if [ "$SCENE_OK" -ne 1 ]; then
    echo "ERROR: Workcell Planning Scene failed to load."
    exit 1
fi


# ============================================================
# 6. Camera TF
# ============================================================

echo
echo "[6/7] Starting camera TF..."

nohup ros2 run tf2_ros static_transform_publisher \
  --x 0.026 \
  --y -0.914 \
  --z 0.790 \
  --roll 0 \
  --pitch 1.5707963 \
  --yaw 0 \
  --frame-id base_link \
  --child-frame-id camera_mount \
  --ros-args \
  -r __node:=camera_mount_tf \
  > "$LOG/camera_mount_tf.log" 2>&1 &


nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0 \
  --roll -1.5707963 \
  --pitch 0 \
  --yaw -1.5707963 \
  --frame-id camera_mount \
  --child-frame-id camera_link \
  --ros-args \
  -r __node:=camera_optical_tf \
  > "$LOG/camera_optical_tf.log" 2>&1 &

sleep 2


# ============================================================
# 7. Vision detector
# ============================================================

echo
echo "[7/7] Starting red target detector..."

nohup python3 \
"$WS/src/aubo_vision_grasp/aubo_vision_grasp/red_target_detector.py" \
--ros-args \
-p use_sim_time:=true \
> "$LOG/red_target_detector.log" 2>&1 &

sleep 2


# ============================================================
# Final check
# ============================================================

echo
echo "=========================================="
echo " SYSTEM STATUS"
echo "=========================================="

echo
echo "Controllers:"
ros2 control list_controllers 2>/dev/null

echo
echo "Suction:"
ros2 service list 2>/dev/null | grep suction || true

echo
echo "MoveIt:"
ros2 service list 2>/dev/null \
    | grep compute_cartesian_path || true

ros2 action list 2>/dev/null \
    | grep -E 'move_action|execute_trajectory' || true

echo
echo "Camera:"
ros2 topic list 2>/dev/null \
    | grep '/camera/workcell_camera' || true

echo
echo "Vision:"
ros2 topic list 2>/dev/null \
    | grep '/vision/' || true

echo
echo "=========================================="
echo " AUBO SYSTEM STARTED"
echo "=========================================="
echo
echo "下一步直接运行："
echo
echo "  ~/aubo_grasp_ws/run_random_pick.sh"
echo
echo "Logs:"
echo "  $LOG"
echo
