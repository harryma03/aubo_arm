#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/aubo_grasp_ws/install/setup.bash

ros2 service call \
/delete_entity \
gazebo_msgs/srv/DeleteEntity \
"{name: 'grasp_cube'}" \
>/dev/null 2>&1 || true

read X Y <<< $(python3 - <<'PY'
import random

x = random.uniform(-0.22, 0.22)
y = random.uniform(-1.02, -0.55)

print(f"{x:.4f} {y:.4f}")
PY
)

echo "=============================="
echo "Random cube"
echo "x = $X"
echo "y = $Y"
echo "=============================="

ros2 run gazebo_ros spawn_entity.py \
  -entity grasp_cube \
  -file /tmp/grasp_cube.sdf \
  -x "$X" \
  -y "$Y" \
  -z 0.08
