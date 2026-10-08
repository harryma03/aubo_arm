#!/usr/bin/env bash
# Public workspace entry point; full simulation and hardware packages are kept.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/lib/env.sh"

usage() {
  cat <<'EOF'
Usage: ./aubo.sh COMMAND [OPTIONS]

  build [colcon options]   Build all workspace packages, including hardware driver
  start [gui|headless]     Start the Gazebo workcell (default: gui)
  stop [--force|--legacy]  Stop workcell processes
  pick [1..6]             Reset objects and run the visual pick queue
  spawn [1..6]            Reset and spawn objects without starting the queue
  setup-network           Install RGB-D socket settings (run with sudo)
  ros <ros2 arguments>    Run ROS CLI with this workspace and configuration loaded
  help                    Show this help

Examples:
  ./aubo.sh build
  ./aubo.sh start gui
  ./aubo.sh pick 4
  ./aubo.sh stop
  ./aubo.sh ros launch aubo_ros2_driver aubo_control.launch.py --show-args

Environment: ROBOT_MODEL (default aubo_i10), AUBO_GRASP_WS, AUBO_CONFIG_FILE.
The ros command forwards hardware-driver arguments unchanged.
EOF
}

start_headless_display() {
  local xvfb_bin="${XVFB_BIN:-$(command -v Xvfb || true)}"
  xvfb_bin="${xvfb_bin:-/tmp/arm_xvfb/root/usr/bin/Xvfb}"
  local display_num="${DISPLAY_NUM:-:99}"
  local pidfile="${XVFB_PIDFILE:-/tmp/arm_xvfb/Xvfb${display_num#:}.pid}"
  local logfile="${XVFB_LOG:-/tmp/arm_xvfb/Xvfb${display_num#:}.log}"
  if [[ ! -x "$xvfb_bin" ]]; then
    echo "ERROR: Xvfb not found; set XVFB_BIN to its executable." >&2
    return 1
  fi
  mkdir -p "$(dirname "$pidfile")" "$(dirname "$logfile")"
  if ! { command -v xdpyinfo >/dev/null && DISPLAY="$display_num" xdpyinfo >/dev/null 2>&1; } &&
     ! { [[ -f "$pidfile" ]] && kill -0 "$(< "$pidfile")" 2>/dev/null; }; then
    nohup "$xvfb_bin" "$display_num" -screen 0 1280x1024x24 +extension GLX -ac > "$logfile" 2>&1 &
    echo $! > "$pidfile"
    sleep 1
  fi
  export DISPLAY="$display_num"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
}

command_name="${1:-help}"
if (( $# )); then shift; fi
case "$command_name" in
  help|-h|--help) usage ;;
  build)
    aubo_load_ros
    cd "$WS"
    exec colcon build "$@"
    ;;
  start)
    mode="${1:-gui}"
    if (( $# > 1 )); then usage >&2; exit 2; fi
    export AUBO_GAZEBO_RVIZ=false
    case "$mode" in
      gui)
        if [[ -z "${DISPLAY:-}" ]] ||
           { command -v xdpyinfo >/dev/null && ! xdpyinfo >/dev/null 2>&1; }; then
          echo "ERROR: No accessible graphical display. Use start headless." >&2
          exit 1
        fi
        export AUBO_GAZEBO_GUI=true AUBO_MOVEIT_RVIZ=true
        ;;
      headless)
        start_headless_display
        export AUBO_GAZEBO_GUI=false AUBO_MOVEIT_RVIZ=false
        ;;
      *) usage >&2; exit 2 ;;
    esac
    exec bash "$WS/scripts/start.sh"
    ;;
  stop) exec bash "$WS/scripts/stop.sh" "$@" ;;
  pick|spawn)
    if (( $# > 1 )) || { (( $# == 1 )) && [[ ! "$1" =~ ^[1-6]$ ]]; }; then
      echo "ERROR: Object count must be an integer from 1 to 6." >&2
      exit 2
    fi
    if (( $# )); then export AUBO_RANDOM_CUBE_COUNT="$1"; fi
    if [[ "$command_name" == pick ]]; then
      exec bash "$WS/scripts/pick.sh"
    fi
    aubo_load_runtime
    exec python3 "$WS/scripts/spawn_cubes.py"
    ;;
  setup-network)
    if (( $# )); then usage >&2; exit 2; fi
    exec bash "$WS/scripts/setup_network.sh"
    ;;
  ros)
    aubo_load_runtime
    exec ros2 "$@"
    ;;
  *) usage >&2; exit 2 ;;
esac
