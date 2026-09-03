#!/bin/bash

WS="$HOME/aubo_grasp_ws"

cd "$WS"

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "======================================"
echo " RANDOM 4-CUBE PICK"
echo "======================================"


# ----------------------------------------------------------
# 1. 检查系统
# ----------------------------------------------------------

echo "[1/3] Checking system..."

if ! ros2 service list 2>/dev/null \
    | grep -q '^/compute_cartesian_path$'; then

    echo "ERROR: MoveIt not running."
    echo "先运行："
    echo "  ~/aubo_grasp_ws/quick_start.sh"
    exit 1
fi

if ! ros2 topic list 2>/dev/null \
    | grep -q '^/vision/target_count$'; then

    echo "ERROR: Vision detector not running."
    echo "先运行："
    echo "  ~/aubo_grasp_ws/quick_start.sh"
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
# 等待物体落稳 + 视觉稳定
# ----------------------------------------------------------

echo
echo "Waiting for target_count = 4..."

FOUND=0

for i in $(seq 1 30); do

    COUNT=$(
        timeout 3 ros2 topic echo \
        --once \
        /vision/target_count \
        2>/dev/null \
        | awk '/data:/ {print $2; exit}'
    )

    echo "target_count=${COUNT:-unknown}"

    if [ "$COUNT" = "4" ]; then
        FOUND=1
        break
    fi

    sleep 1
done


if [ "$FOUND" -ne 1 ]; then

    echo "ERROR: Could not detect 4 objects."
    exit 1
fi


# ----------------------------------------------------------
# 3. 自动全部抓取
# ----------------------------------------------------------

echo
echo "[3/3] Starting automatic picking..."

python3 \
"$WS/src/aubo_vision_grasp/aubo_vision_grasp/visual_pick_all.py" \
--ros-args \
-p use_sim_time:=true
