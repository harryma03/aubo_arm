#!/usr/bin/env bash

set -euo pipefail

WS="${AUBO_GRASP_WS:-/home/ltt/arm_grap_ws_jaka}"
XVFB_BIN="${XVFB_BIN:-/tmp/arm_xvfb/root/usr/bin/Xvfb}"
DISPLAY_NUM="${DISPLAY_NUM:-:99}"
XVFB_PIDFILE="${XVFB_PIDFILE:-/tmp/arm_xvfb/Xvfb${DISPLAY_NUM#:}.pid}"
XVFB_LOG="${XVFB_LOG:-/tmp/arm_xvfb/Xvfb${DISPLAY_NUM#:}.log}"

if [[ ! -x "$XVFB_BIN" ]]; then
  echo "ERROR: Xvfb not found: $XVFB_BIN" >&2
  echo "Set XVFB_BIN to an existing Xvfb binary or install Xvfb separately." >&2
  exit 1
fi

mkdir -p "$(dirname "$XVFB_PIDFILE")"

if { command -v xdpyinfo >/dev/null 2>&1 && DISPLAY="$DISPLAY_NUM" xdpyinfo >/dev/null 2>&1; } \
  || { [[ -f "$XVFB_PIDFILE" ]] && kill -0 "$(cat "$XVFB_PIDFILE")" 2>/dev/null; }; then
  echo "Using existing Xvfb on $DISPLAY_NUM."
else
  nohup "$XVFB_BIN" "$DISPLAY_NUM" \
    -screen 0 1280x1024x24 \
    +extension GLX -ac \
    > "$XVFB_LOG" 2>&1 &
  echo $! > "$XVFB_PIDFILE"
  sleep 1
fi

export DISPLAY="$DISPLAY_NUM"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export AUBO_GAZEBO_GUI=false
export AUBO_GAZEBO_RVIZ=false
export AUBO_MOVEIT_RVIZ=false

exec "$WS/start_aubo_system.sh"
