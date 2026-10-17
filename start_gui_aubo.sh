#!/usr/bin/env bash

set -euo pipefail

WS="${AUBO_GRASP_WS:-/home/ltt/arm_grap_ws_jaka}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set. Run this script from a graphical desktop session." >&2
  exit 1
fi

if command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
  echo "ERROR: Cannot connect to the current display: $DISPLAY" >&2
  echo "Run this script in the desktop terminal that owns the graphical session." >&2
  exit 1
fi

# One Gazebo window plus one MoveIt RViz window.
# The RViz started by aubo_gazebo is disabled to avoid a duplicate RViz window.
export AUBO_GAZEBO_GUI=true
export AUBO_GAZEBO_RVIZ=false
export AUBO_MOVEIT_RVIZ=true
# Public robot-model interface. Override only this value when changing arms:
#   ROBOT_MODEL=jaka_Zu12 ./start_gui_aubo.sh
# The default workcell robot is now JAKA Zu12.
export ROBOT_MODEL="jaka_Zu12"
# Package compatibility alias; JAKA Zu12 is the only installed model.
export AUBO_MOVEIT_MODEL="jaka_Zu12"


exec "$WS/start_aubo_system.sh"
