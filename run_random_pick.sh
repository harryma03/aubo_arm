#!/bin/bash

set -e

WS="$HOME/aubo_grasp_ws"

cd "$WS"

conda deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "=========================================="
echo " RANDOM 4-CUBE VISUAL PICK"
echo "=========================================="

# ============================================================
# 1. 检查系统
# ============================================================

echo
echo "[1/4] Checking system..."

if ! ros2 service list 2>/dev/null \
    | grep -q '^/compute_cartesian_path$'; then

    echo "ERROR: MoveIt is not running."
    echo "Please run:"
    echo "  ~/aubo_grasp_ws/start_aubo_system.sh"
    exit 1
fi

if ! ros2 service list 2>/dev/null \
    | grep -q '^/suction_on$'; then

    echo "ERROR: suction plugin is not running."
    exit 1
fi

if ! ros2 topic list 2>/dev/null \
    | grep -q '^/vision/target_count$'; then

    echo "ERROR: red_target_detector is not running."
    exit 1
fi

echo "System OK."


# ============================================================
# 2. 生成新的随机4物体
#
# spawn_random_cubes.py 自己负责：
# suction_off
# 删除旧物体
# 重新随机生成4个物体
# ============================================================

echo
echo "[2/4] Resetting and spawning 4 random cubes..."

timeout 30 python3 \
"$WS/spawn_random_cubes.py"

echo "Random cubes spawned."


# ============================================================
# 3. 等待视觉稳定
# ============================================================

echo
echo "[3/4] Waiting for vision target_count = 4..."

FOUND=0

for i in $(seq 1 30); do

    COUNT=$(
        timeout 3 ros2 topic echo \
        --once \
        /vision/target_count \
        2>/dev/null \
        | awk '/data:/ {print $2; exit}'
    ) || true

    echo "  target_count = ${COUNT:-unknown}"

    if [ "$COUNT" = "4" ]; then
        FOUND=1
        break
    fi

    sleep 1
done

if [ "$FOUND" -ne 1 ]; then
    echo
    echo "ERROR: Vision did not detect all 4 cubes."
    echo
    echo "Check:"
    echo "  ros2 topic echo --once /vision/target_count"
    exit 1
fi

echo "Vision ready."


# ============================================================
# 4. 自动全部抓取
# ============================================================

echo
echo "[4/4] Starting automatic visual picking..."
echo

python3 \
"$WS/src/aubo_vision_grasp/aubo_vision_grasp/visual_pick_all.py" \
--ros-args \
-p use_sim_time:=true

echo
echo "=========================================="
echo " RANDOM PICK FINISHED"
echo "=========================================="
