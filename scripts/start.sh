#!/bin/bash

set -e

source "$(dirname "${BASH_SOURCE[0]}")/lib/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/lib/gazebo_guard.sh"
aubo_load_runtime
LOG="$WS/run_logs"
GAZEBO_GUI="${AUBO_GAZEBO_GUI:-false}"
GAZEBO_RVIZ="${AUBO_GAZEBO_RVIZ:-false}"
MOVEIT_RVIZ="${AUBO_MOVEIT_RVIZ:-true}"

mkdir -p "$LOG"

cd "$WS"

ROBOT_MODEL="${ROBOT_MODEL:-${AUBO_MOVEIT_MODEL:-aubo_i10}}"
case "$ROBOT_MODEL" in
  aubo_i5|aubo_C3|aubo_i10|aubo_i10_30|aubo_i16|aubo_i12)
    ;;
  *)
    echo "ERROR: Unsupported ROBOT_MODEL: $ROBOT_MODEL" >&2
    echo "Supported models: aubo_i5, aubo_C3, aubo_i10, aubo_i10_30, aubo_i16, aubo_i12" >&2
    exit 1
    ;;
esac

# Existing package launch arguments still use the historical AUBO name. Keep
# this internal alias at the boundary while exposing ROBOT_MODEL to users.
export ROBOT_MODEL
export AUBO_MOVEIT_MODEL="$ROBOT_MODEL"

echo "=========================================="
echo " ROBOT $ROBOT_MODEL WORKCELL AUTO START"
echo "=========================================="
echo "UI: Gazebo=$GAZEBO_GUI | Gazebo RViz=$GAZEBO_RVIZ | MoveIt RViz=$MOVEIT_RVIZ"

WORKCELL_X="$AUBO_WORKCELL_X"
WORKCELL_Y="$AUBO_WORKCELL_Y"
WORKCELL_Z="$AUBO_WORKCELL_Z"
HOPPER_LOCAL_X="$AUBO_HOPPER_LOCAL_X"
HOPPER_LOCAL_Y="$AUBO_HOPPER_LOCAL_Y"
HOPPER_LOCAL_Z="$AUBO_HOPPER_LOCAL_Z"
GROUND_X="$AUBO_GROUND_X"
GROUND_Y="$AUBO_GROUND_Y"
GROUND_Z="$AUBO_GROUND_Z"

echo "WORKCELL POSE: x=$WORKCELL_X | y=$WORKCELL_Y | z=$WORKCELL_Z"
echo "HOPPER LOCAL POSE: x=$HOPPER_LOCAL_X | y=$HOPPER_LOCAL_Y | z=$HOPPER_LOCAL_Z"

export GAZEBO_MODEL_DATABASE_URI=""

export GAZEBO_MODEL_PATH="$WS/install/aubo_workcell_description/share/aubo_workcell_description/models:$WS/install/aubo_description/share:/opt/ros/humble/share${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

export GAZEBO_PLUGIN_PATH="$WS/install/suction_gripper_gazebo/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"


# ============================================================
# 0. 清理旧进程
# ============================================================

echo
echo "[0/6] Cleaning old processes..."

# Keep cleanup scoped to this workcell.  In particular, do not kill every
# gzserver, RViz, MoveIt, or ROS node belonging to other projects.
"$WS/scripts/stop.sh"
gazebo_check_port

# Only stop launch trees created by this invocation on a failed startup.
STARTUP_LAUNCH_PIDS=()
startup_exit() {
  local status=$? pid
  if (( status != 0 )); then
    for pid in "${STARTUP_LAUNCH_PIDS[@]}"; do
      gazebo_stop_launch_tree "$pid"
    done
  fi
}
trap startup_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM


# ============================================================
# 1. Gazebo + AUBO + suction
# ============================================================

echo
echo "[1/6] Starting Gazebo + $AUBO_MOVEIT_MODEL + suction..."

nohup ros2 launch \
  aubo_gazebo \
  aubo_gazebo.launch.py \
  robot_type:="$AUBO_MOVEIT_MODEL" \
  launch_rviz:="$GAZEBO_RVIZ" \
  gazebo_gui:="$GAZEBO_GUI" \
  > "$LOG/gazebo.log" 2>&1 &
GAZEBO_LAUNCH_PID=$!
STARTUP_LAUNCH_PIDS+=("$GAZEBO_LAUNCH_PID")

echo "Waiting for Gazebo..."

for i in $(seq 1 60); do

    gazebo_check_alive
    if gazebo_has_server && timeout 3 ros2 service list 2>/dev/null \
        | grep -q '^/spawn_entity$'; then

        echo "Gazebo ready."
        gazebo_check_alive
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
echo "[2/6] Spawning workcell..."

nohup ros2 launch \
  aubo_workcell_description \
  spawn_workcell.launch.py \
  workcell_x:="$WORKCELL_X" \
  workcell_y:="$WORKCELL_Y" \
  workcell_z:="$WORKCELL_Z" \
  ground_x:="$GROUND_X" \
  ground_y:="$GROUND_Y" \
  ground_z:="$GROUND_Z" \
  > "$LOG/workcell.log" 2>&1 &
STARTUP_LAUNCH_PIDS+=("$!")

echo "Waiting for RGB-D camera..."

for i in $(seq 1 30); do

    gazebo_check_alive
    if timeout 3 ros2 topic list 2>/dev/null \
        | grep -Fxq "$AUBO_DEPTH_TOPIC"; then

        echo "Workcell + RGB-D camera ready."
        break
    fi

    sleep 1

    if [ "$i" -eq 30 ]; then
        echo "ERROR: Camera topic not detected; aborting startup."
        echo "Check:"
        echo "  tail -100 $LOG/workcell.log"
        exit 1
    fi
done


# ============================================================
# 3. Controller
# ============================================================

echo
echo "[3/6] Checking trajectory controller..."

CONTROLLER_MANAGER_READY=0

for i in $(seq 1 30); do

    gazebo_check_alive

    if timeout 3 ros2 service list 2>/dev/null \
        | grep -q '^/controller_manager/list_controllers$'; then
        CONTROLLER_MANAGER_READY=1
        break
    fi

    sleep 1
done

if [ "$CONTROLLER_MANAGER_READY" -ne 1 ]; then
    echo "ERROR: controller_manager did not become available."
    echo "Check:"
    echo "  grep -n 'robot_state_publisher service\|gazebo_ros2_control' $LOG/gazebo.log | tail -30"
    exit 1
fi

CONTROLLERS="$(timeout 5 ros2 control list_controllers 2>/dev/null || true)"

if printf '%s\n' "$CONTROLLERS" \
    | grep joint_trajectory_controller \
    | grep -q active; then

    echo "joint_trajectory_controller already active."

else

    echo "Activating joint_trajectory_controller..."

    if ! timeout 20 ros2 control load_controller \
        --set-state active \
        joint_trajectory_controller; then
        echo "ERROR: failed to activate joint_trajectory_controller."
        echo "Check: ros2 control list_controllers"
        exit 1
    fi
fi


# ============================================================
# 4. MoveIt
# ============================================================

echo
echo "[4/6] Starting MoveIt2..."

nohup ros2 launch \
  aubo_moveit_config \
  aubo_moveit.launch.py \
  aubo_type:="$AUBO_MOVEIT_MODEL" \
  launch_rviz:="$MOVEIT_RVIZ" \
  > "$LOG/moveit.log" 2>&1 &
STARTUP_LAUNCH_PIDS+=("$!")

echo "Waiting for MoveIt..."

for i in $(seq 1 60); do

    SERVICE_OK=0
    gazebo_check_alive
    ACTION_OK=0

    timeout 3 ros2 service list 2>/dev/null \
        | grep -q '^/compute_cartesian_path$' \
        && SERVICE_OK=1

    timeout 3 ros2 action list 2>/dev/null \
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
echo "[5/6] Waiting for Planning Scene..."

# MoveIt services may appear before PlanningSceneMonitor is fully ready.
sleep 5

SCENE_SCRIPT="$WS/src/aubo_vision_grasp/aubo_vision_grasp/setup_workcell_scene.py"

SCENE_OK=0

for attempt in 1 2 3; do

    echo "Applying Planning Scene... attempt $attempt/3"

    APPLY_OK=0
    if python3 \
      "$SCENE_SCRIPT" \
      --ros-args \
      -p use_sim_time:=true \
      -p workcell_x:="$WORKCELL_X" \
      -p workcell_y:="$WORKCELL_Y" \
      -p workcell_z:="$WORKCELL_Z" \
      -p hopper_local_x:="$HOPPER_LOCAL_X" \
      -p hopper_local_y:="$HOPPER_LOCAL_Y" \
      -p hopper_local_z:="$HOPPER_LOCAL_Z" \
      -p ground_x:="$GROUND_X" \
      -p ground_y:="$GROUND_Y" \
      -p ground_z:="$GROUND_Z"
    then
        APPLY_OK=1
    fi

    sleep 2

    SCENE_RESPONSE=$(
      ros2 service call \
        /get_planning_scene \
        moveit_msgs/srv/GetPlanningScene \
        "{components: {components: 24}}" \
        2>/dev/null || true
    )

    REQUIRED_SCENE_IDS=(
      workcell_ground
      workcell_pedestal
      workcell_box
      workcell_store
      camera_holder_post
      camera_holder_top_beam
      camera_holder_camera_head
      camera_holder_base
      camera_holder_top_support
      camera_holder_curved_support
    )
    MISSING_SCENE_IDS=()
    for object_id in "${REQUIRED_SCENE_IDS[@]}"; do
        if ! grep -Fq "id='$object_id'" <<< "$SCENE_RESPONSE"; then
            MISSING_SCENE_IDS+=("$object_id")
        fi
    done

    if [ "$APPLY_OK" -eq 1 ] && [ "${#MISSING_SCENE_IDS[@]}" -eq 0 ]; then
        echo "Planning Scene ready: all 10 required collision objects verified."
        SCENE_OK=1
        break
    fi

    if [ "${#MISSING_SCENE_IDS[@]}" -gt 0 ]; then
        echo "Missing Planning Scene objects: ${MISSING_SCENE_IDS[*]}"
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
echo "[6/6] Starting camera TF..."

CAMERA_MOUNT_X="$(awk -v a="$WORKCELL_X" -v b="${AUBO_CAMERA_LOCAL_X:-0.126}" 'BEGIN { printf "%.6f", a + b }')"
CAMERA_MOUNT_Y="$(awk -v a="$WORKCELL_Y" -v b="${AUBO_CAMERA_LOCAL_Y:--0.914}" 'BEGIN { printf "%.6f", a + b }')"
CAMERA_MOUNT_Z="$(awk -v a="$WORKCELL_Z" -v b="${AUBO_CAMERA_LOCAL_Z:-1.740}" 'BEGIN { printf "%.6f", a + b }')"

nohup ros2 run tf2_ros static_transform_publisher \
  --x "$CAMERA_MOUNT_X" \
  --y "$CAMERA_MOUNT_Y" \
  --z "$CAMERA_MOUNT_Z" \
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
echo "=========================================="
echo " AUBO SYSTEM STARTED"
echo "=========================================="
echo
echo "下一步直接运行："
echo
echo "  $WS/aubo.sh pick"
echo "  （视觉检测器和动态队列将在抓取期间统一启动、统一退出）"
echo
echo "Logs:"
echo "  $LOG"
echo
