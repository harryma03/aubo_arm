#!/usr/bin/env bash

# Stop the processes started by this workspace without killing unrelated ROS 2
# projects.  Use --force only when the normal SIGTERM shutdown did not clear
# the graph after a few seconds.


set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/lib/env.sh"
LOG="$WS/run_logs"
FORCE=false

usage() {
  cat <<'EOF'
Usage: ./aubo.sh stop [--force] [--legacy]

Stops this AUBO/Gazebo workcell only:
  Gazebo + workcell + MoveIt launch parents
  children of those launch parents, including robot_state_publisher
  gzserver/gzclient whose working directory is this workspace
  red-target detectors and visual-pick programs
  legacy vision detector/supervisor processes left by older startup flows
  project-named static TF publishers

Processes first receive SIGTERM and have 3 seconds to exit.  Remaining
workspace-scoped Gazebo/MoveIt/RViz processes then receive SIGKILL.
With --force, any other remaining matching project processes also receive
SIGKILL.
With --legacy, also stop an old unnamed robot_state_publisher whose working
directory is this workspace.  This is for processes left by older/manual
startup flows; inspect pgrep -af first if other projects use this workspace.
EOF
}

LEGACY=false
while (( $# > 0 )); do
  case "$1" in
    --force) FORCE=true ;;
    --legacy) LEGACY=true ;;
    -h|--help) usage; exit 0 ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
  shift
done

stop_pid_file() {
  local pid_file="$1"
  local label="$2"
  local pid=""
  local cmdline=""

  [[ -r "$pid_file" ]] || return 0
  IFS= read -r pid < "$pid_file" || true

  if [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 "$pid" 2>/dev/null; then
    cmdline="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    # A PID file can be stale and its numeric PID can be reused.  Only signal
    # it when it still belongs to this workspace's vision process.
    if [[ "$cmdline" == *aubo_vision_grasp* && "$cmdline" == *red_target_detector* ]]; then
      echo "Stopping $label (PID $pid)"
      # Detectors are launched with setsid; stop the whole session when the
      # PID is also its process-group leader, then fall back to the PID.
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    else
      echo "Ignoring stale PID file: $pid_file"
    fi
  fi

  rm -f "$pid_file"
}

stop_supervisor_pid_file() {
  local pid_file="$1"
  local pid=""
  local cmdline=""

  [[ -r "$pid_file" ]] || return 0
  IFS= read -r pid < "$pid_file" || true

  if [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 "$pid" 2>/dev/null; then
    cmdline="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    # The supervisor PID is workspace-scoped and its command line is stable;
    # do not signal a reused PID belonging to another process.
    if [[ "$cmdline" == *"$WS/vision_detector_supervisor.sh"* ]]; then
      echo "Stopping pick vision supervisor (PID $pid)"
      kill -TERM "$pid" 2>/dev/null || true
    else
      echo "Ignoring stale PID file: $pid_file"
    fi
  fi

  rm -f "$pid_file"
}

signal_pattern() {
  local signal="$1"
  local pattern="$2"
  local label="$3"

  if pgrep -f -- "$pattern" >/dev/null 2>&1; then
    echo "$signal stopping $label"
    pkill "-$signal" -f -- "$pattern" 2>/dev/null || true
  fi
}

# ros2 launch starts several child processes.  Stopping only the launch
# command can leave robot_state_publisher, move_group, or static TF children
# alive.  Walk the process tree and stop children before their launch parent.
collect_descendants() {
  local parent="$1"
  local child=""

  while IFS= read -r child; do
    collect_descendants "$child"
    printf '%s\n' "$child"
  done < <(pgrep -P "$parent" 2>/dev/null || true)
}

signal_tree_pattern() {
  local signal="$1"
  local pattern="$2"
  local label="$3"
  local root="" pid=""
  local -a roots=()

  mapfile -t roots < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  for root in "${roots[@]}"; do
    [[ "$root" == "$$" ]] && continue
    while IFS= read -r pid; do
      [[ "$pid" == "$$" ]] && continue
      if kill -0 "$pid" 2>/dev/null; then
        echo "$signal stopping $label child (PID $pid)"
        kill "-$signal" "$pid" 2>/dev/null || true
      fi
    done < <(collect_descendants "$root")
    if kill -0 "$root" 2>/dev/null; then
      echo "$signal stopping $label (PID $root)"
      kill "-$signal" "$root" 2>/dev/null || true
    fi
  done
}

# Gazebo plugins and ros2_control nodes live inside gzserver.  If the launch
# parent has already exited, the Gazebo process can be re-parented and no
# longer be reachable through the launch tree.  Scope this fallback by the
# workspace working directory instead of using a global pkill gzserver.
handle_workspace_gazebo() {
  local signal="$1"
  local process_name=""
  local pid=""
  local cwd=""

  for process_name in gzserver gzclient; do
    while IFS= read -r pid; do
      [[ -n "$pid" && "$pid" != "$$" ]] || continue
      cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null || true)"

      if [[ "$cwd" == "$WS" ]]; then
        echo "${signal/-/} stopping workspace $process_name (PID $pid)"
        kill "$signal" "$pid" 2>/dev/null || true
      fi
    done < <(pgrep -x "$process_name" 2>/dev/null || true)
  done
}

# A launch parent can disappear before its MoveIt/RViz children.  Those
# orphaned processes no longer belong to a launch tree, so stop only matching
# executables whose working directory is this workspace.  In particular,
# moveit_simple_controller_manager and transform_listener_impl are nodes
# hosted inside move_group; stopping move_group removes all of them.
handle_workspace_ros_processes() {
  local signal="$1"
  local process_name=""
  local pid=""
  local cwd=""

  for process_name in move_group robot_state_publisher rviz2; do
    while IFS= read -r pid; do
      [[ -n "$pid" && "$pid" != "$$" ]] || continue
      cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null || true)"

      if [[ "$cwd" == "$WS" ]]; then
        echo "${signal/-/} stopping workspace $process_name (PID $pid)"
        kill "$signal" "$pid" 2>/dev/null || true
      fi
    # Use the full command line because Linux truncates process names longer
    # than 15 characters (notably robot_state_publisher).
    done < <(
      pgrep -f -- "(^|/)$process_name([[:space:]]|$)" 2>/dev/null || true
    )
  done
}

# ros2 node list normally reads a per-user discovery daemon.  After all real
# processes are gone the daemon can still show their old graph entries for a
# while.  Stopping only the CLI daemon is safe for running ROS nodes and makes
# the next ros2 command start with a fresh graph cache.
reset_ros2_cli_daemon() {
  command -v ros2 >/dev/null 2>&1 || return 0

  echo 'Refreshing ROS 2 CLI discovery cache'
  if timeout --signal=TERM --kill-after=1s 5s \
      ros2 daemon stop >/dev/null 2>&1; then
    return 0
  fi

  # A wedged daemon can block both `aubo.sh stop` and the cleanup phase of
  # `aubo.sh start`. Restrict the fallback to this user's daemon for the
  # active ROS domain; never match arbitrary Python or ROS processes.
  local domain_id="${ROS_DOMAIN_ID:-0}"
  local pid=""
  local cmdline=""
  local -a stale_daemon_pids=()
  echo "WARNING: ROS 2 CLI daemon did not stop within 5 seconds; removing the stale domain-$domain_id daemon."
  while IFS= read -r pid; do
    [[ -n "$pid" && "$pid" != "$$" ]] || continue
    cmdline="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    if [[ "$cmdline" == *'ros2cli.daemon.daemonize'* &&
          "$cmdline" == *'--name ros2-daemon'* &&
          "$cmdline" == *"--ros-domain-id $domain_id"* ]]; then
      kill -TERM "$pid" 2>/dev/null || true
      stale_daemon_pids+=("$pid")
    fi
  done < <(pgrep -u "$(id -u)" -f 'ros2cli[.]daemon[.]daemonize' 2>/dev/null || true)

  if (( ${#stale_daemon_pids[@]} )); then
    sleep 1
    for pid in "${stale_daemon_pids[@]}"; do
      kill -0 "$pid" 2>/dev/null || continue
      kill -KILL "$pid" 2>/dev/null || true
    done
  fi
}

# Older/manual launches used the default node name robot_state_publisher.
# It cannot be identified from the ROS graph alone, so --legacy scopes this
# fallback to processes whose working directory is this workspace.
handle_legacy_robot_state_publishers() {
  local signal="$1"
  local pid=""
  local cmdline=""
  local cwd=""

  while IFS= read -r pid; do
    [[ -n "$pid" && "$pid" != "$$" ]] || continue
    cmdline="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null || true)"

    if [[ "$cwd" == "$WS" && "$cmdline" == *robot_state_publisher* ]]; then
      echo "${signal/-/} stopping legacy robot_state_publisher (PID $pid)"
      kill "$signal" "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -f -- 'robot_state_publisher' 2>/dev/null || true)
}

declare -a TREE_PATTERNS=(
  'ros2 launch aubo_gazebo aubo_gazebo.launch.py'
  'ros2 launch aubo_workcell_description spawn_workcell.launch.py'
  'ros2 launch aubo_moveit_config aubo_moveit.launch.py'
  'ros2 launch aubo_vision_grasp visual_pick_system.launch.py'
)

declare -a TREE_LABELS=(
  'Gazebo/AUBO launch tree'
  'workcell launch tree'
  'MoveIt launch tree'
  'visual-pick launch tree'
)

declare -a PATTERNS=(
  'aubo_vision_grasp.*red_target_detector'
  'aubo_vision_grasp.*visual_pick_all'
  'aubo_vision_grasp.*visual_pick_queue'
  'aubo_vision_grasp.*visual_pick_to_bin'
  'aubo_vision_grasp.*auto_suction_grasp'
  '__node:=aubo_world_base_tf'
  'tf2_ros.*static_transform_publisher.*world.*base_link'
  '__node:=camera_mount_tf'
  '__node:=camera_optical_tf'
)

declare -a LABELS=(
  'red-target detector(s)'
  'visual_pick_all program'
  'visual_pick_queue program'
  'visual_pick_to_bin program'
  'auto_suction_grasp program'
  'AUBO world-to-base static TF'
  'legacy world-to-base static TF'
  'camera_mount_tf publisher'
  'camera_optical_tf publisher'
)

echo '=========================================='
echo ' CLEAN AUBO i10 WORKCELL PROCESSES'
echo '=========================================='

stop_pid_file "$LOG/red_target_detector.pid" 'runtime vision detector'
stop_pid_file "$LOG/red_target_detector_pick.pid" 'pick vision detector'
stop_supervisor_pid_file "$LOG/red_target_detector_pick_supervisor.pid"

if "$LEGACY"; then
  handle_legacy_robot_state_publishers -TERM
fi

handle_workspace_gazebo -TERM
handle_workspace_ros_processes -TERM

for i in "${!TREE_PATTERNS[@]}"; do
  signal_tree_pattern TERM "${TREE_PATTERNS[$i]}" "${TREE_LABELS[$i]}"
done

for i in "${!PATTERNS[@]}"; do
  signal_pattern TERM "${PATTERNS[$i]}" "${LABELS[$i]}"
done

signal_pattern TERM "$WS/vision_detector_supervisor.sh" 'pick vision supervisor'

echo 'Waiting up to 3 seconds for a graceful shutdown...'
sleep 3

# These fallbacks are restricted by /proc/<pid>/cwd == "$WS", so it is safe
# to make the cleanup reliable without requiring --force.  Gazebo in
# particular can remain alive after its launch parent has already exited.
handle_workspace_gazebo -KILL
handle_workspace_ros_processes -KILL

if "$FORCE"; then
  if "$LEGACY"; then
    handle_legacy_robot_state_publishers -KILL
  fi

  handle_workspace_gazebo -KILL
  handle_workspace_ros_processes -KILL

  for i in "${!TREE_PATTERNS[@]}"; do
    signal_tree_pattern KILL "${TREE_PATTERNS[$i]}" "${TREE_LABELS[$i]}"
  done

  for i in "${!PATTERNS[@]}"; do
    signal_pattern KILL "${PATTERNS[$i]}" "${LABELS[$i]}"
  done

  signal_pattern KILL "$WS/vision_detector_supervisor.sh" 'pick vision supervisor'
fi

reset_ros2_cli_daemon

echo
echo 'Cleanup command finished.'
echo 'robot_description is a parameter/topic owned by ROS nodes; it disappears after its owner nodes stop.'
echo 'The ROS 2 CLI discovery cache was reset; the next ros2 command rebuilds it.'
echo 'Verify the live graph with: ros2 node list --no-daemon'
