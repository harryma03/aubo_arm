#!/bin/bash

source /opt/ros/humble/setup.bash
WS="${AUBO_GRASP_WS:-/home/ltt/arm_grap_ws_jaka}"
source "$WS/install/setup.bash"

LAYOUT_ENV="$WS/src/aubo_workcell_description/config/workcell_layout.env"
if [ -f "$LAYOUT_ENV" ]; then
  source "$LAYOUT_ENV"
fi

CUBE_MODEL_FILE="${CUBE_MODEL_FILE:-$WS/models/grasp_cube/model.sdf}"

if [ ! -f "$CUBE_MODEL_FILE" ]; then
  echo "ERROR: Cube model not found: $CUBE_MODEL_FILE" >&2
  exit 1
fi

ros2 service call \
/delete_entity \
gazebo_msgs/srv/DeleteEntity \
"{name: 'grasp_cube'}" \
>/dev/null 2>&1 || true

read X Y Z <<< $(AUBO_WORKCELL_X="${AUBO_WORKCELL_X:-0.0}" AUBO_WORKCELL_Y="${AUBO_WORKCELL_Y:-0.0}" AUBO_WORKCELL_Z="${AUBO_WORKCELL_Z:--0.95}" AUBO_HOPPER_LOCAL_X="${AUBO_HOPPER_LOCAL_X:-0.00000335}" AUBO_HOPPER_LOCAL_Y="${AUBO_HOPPER_LOCAL_Y:--0.75005309}" python3 - <<'PY'
import random
import os

workcell_x = float(os.environ['AUBO_WORKCELL_X'])
workcell_y = float(os.environ['AUBO_WORKCELL_Y'])
workcell_z = float(os.environ['AUBO_WORKCELL_Z'])
hopper_x = workcell_x + float(os.environ['AUBO_HOPPER_LOCAL_X'])
hopper_y = workcell_y + float(os.environ['AUBO_HOPPER_LOCAL_Y'])
x = hopper_x + random.uniform(-0.22, 0.22)
y = hopper_y + random.uniform(-0.27, 0.20)
z = 0.08 + (workcell_z - (-0.95))

print(f"{x:.4f} {y:.4f} {z:.4f}")
PY
)

echo "=============================="
echo "Random cube"
echo "x = $X"
echo "y = $Y"
echo "z = $Z"
echo "=============================="

ros2 run gazebo_ros spawn_entity.py \
  -entity grasp_cube \
  -file "$CUBE_MODEL_FILE" \
  -x "$X" \
  -y "$Y" \
  -z "$Z"
