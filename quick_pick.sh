#!/bin/bash

WS="${AUBO_GRASP_WS:-/home/ltt/arm_grap_ws_jaka}"

cd "$WS"

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

LAYOUT_ENV="$WS/src/aubo_workcell_description/config/workcell_layout.env"
if [ -f "$LAYOUT_ENV" ]; then
    source "$LAYOUT_ENV"
fi
export AUBO_WORKCELL_X="${AUBO_WORKCELL_X:-0.0}"
export AUBO_WORKCELL_Y="${AUBO_WORKCELL_Y:-0.0}"
export AUBO_WORKCELL_Z="${AUBO_WORKCELL_Z:--0.95}"
export AUBO_HOPPER_LOCAL_X="${AUBO_HOPPER_LOCAL_X:-0.00000335}"
export AUBO_HOPPER_LOCAL_Y="${AUBO_HOPPER_LOCAL_Y:--0.75005309}"
export AUBO_HOPPER_LOCAL_Z="${AUBO_HOPPER_LOCAL_Z:-0.93443794}"
export AUBO_HOPPER_BOTTOM_SIZE_X="${AUBO_HOPPER_BOTTOM_SIZE_X:-0.615}"
export AUBO_HOPPER_BOTTOM_SIZE_Y="${AUBO_HOPPER_BOTTOM_SIZE_Y:-0.930}"
export AUBO_CUBE_SIZE_X="${AUBO_CUBE_SIZE_X:-0.150}"
export AUBO_CUBE_SIZE_Y="${AUBO_CUBE_SIZE_Y:-0.100}"
export AUBO_HOPPER_WALL_MARGIN="${AUBO_HOPPER_WALL_MARGIN:-0.020}"
export AUBO_TARGET_STABLE_TIME="${AUBO_TARGET_STABLE_TIME:-1.20}"
export AUBO_TARGET_EMPTY_STABLE_TIME="${AUBO_TARGET_EMPTY_STABLE_TIME:-2.00}"
export AUBO_TARGET_MATCH_DISTANCE="${AUBO_TARGET_MATCH_DISTANCE:-0.120}"
export AUBO_TARGET_MISSING_CONFIRMATIONS="${AUBO_TARGET_MISSING_CONFIRMATIONS:-3}"
export AUBO_TARGET_MAX_RETRIES="${AUBO_TARGET_MAX_RETRIES:-1}"
export AUBO_TARGET_SPLIT_ENABLE="${AUBO_TARGET_SPLIT_ENABLE:-0}"
export AUBO_TARGET_SPLIT_PEAK_RATIO="${AUBO_TARGET_SPLIT_PEAK_RATIO:-0.45}"
export AUBO_TARGET_SPLIT_MIN_AREA_RATIO="${AUBO_TARGET_SPLIT_MIN_AREA_RATIO:-1.55}"

PICK_SCHEDULER="${AUBO_PICK_SCHEDULER:-queue}"

echo "======================================"
echo " RANDOM 4-CUBE PICK"
echo "======================================"


# ----------------------------------------------------------
# 1. 检查系统
# ----------------------------------------------------------

echo "[1/3] Checking system..."

if ! ros2 service list --no-daemon 2>/dev/null \
    | grep -q '^/compute_cartesian_path$'; then

    echo "ERROR: MoveIt not running."
    echo "先运行："
    echo "  $WS/start_aubo_system.sh"
    exit 1
fi

if [ "$PICK_SCHEDULER" = "legacy" ]; then
    REQUIRED_VISION_TOPIC='^/vision/target_count$'
else
    REQUIRED_VISION_TOPIC='^/vision/target_array$'
fi

if ! ros2 topic list --no-daemon 2>/dev/null \
    | grep -q "$REQUIRED_VISION_TOPIC"; then

    echo "ERROR: Vision detector not running."
    echo "先运行："
    echo "  $WS/start_aubo_system.sh"
    exit 1
fi


# ----------------------------------------------------------
# 2. 随机生成4个
# ----------------------------------------------------------

echo "[2/3] Reset + spawn 4 random cubes..."

python3 \
"$WS/spawn_random_cubes.py"

if [ $? -ne 0 ]; then

    echo "ERROR: spawn_random_cubes.py failed."
    exit 1
fi


# ----------------------------------------------------------
# 3. 自动动态抓取
# ----------------------------------------------------------

echo
echo "[3/3] Starting automatic picking..."

if [ "$PICK_SCHEDULER" = "legacy" ]; then
    python3 \
        "$WS/src/aubo_vision_grasp/aubo_vision_grasp/visual_pick_all.py" \
        --ros-args \
        -p use_sim_time:=true
else
    ros2 run \
        aubo_vision_grasp \
        visual_pick_queue \
        --ros-args \
        -p use_sim_time:=true
fi
