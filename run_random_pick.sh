#!/usr/bin/env bash

# One-shot random-cube visual picking entry point.
# Lifecycle ownership is intentionally simple:
#   this script -> one ROS launch -> detector + queue

set -eo pipefail

WS="${AUBO_GRASP_WS:-/home/ltt/arm_grap_ws_jaka}"
LOG="$WS/run_logs"
PICK_LOG="$LOG/visual_pick_system.log"

cd "$WS"
mkdir -p "$LOG"

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/load_aubo_config.sh"

set -u

# Single public interface for the random-scene size.  The spawning script,
# cleanup range and user-facing status all consume this value.
CUBE_COUNT="${AUBO_RANDOM_CUBE_COUNT:-6}"
if [[ ! "$CUBE_COUNT" =~ ^[1-6]$ ]]; then
  echo "ERROR: AUBO_RANDOM_CUBE_COUNT must be an integer from 1 to 6."
  exit 1
fi
export AUBO_RANDOM_CUBE_COUNT="$CUBE_COUNT"

# Raw 640x480 RGB-D frames need a larger UDP receive window than the Linux
# default. Fail with an actionable message instead of occasionally accepting
# one frame and then timing out inside the detector.
DDS_REQUIRED_SOCKET_BUFFER=10485760
DDS_RMEM_MAX="$(< /proc/sys/net/core/rmem_max)"
DDS_WMEM_MAX="$(< /proc/sys/net/core/wmem_max)"

if (( DDS_RMEM_MAX < DDS_REQUIRED_SOCKET_BUFFER \
      || DDS_WMEM_MAX < DDS_REQUIRED_SOCKET_BUFFER )); then
  echo "ERROR: Linux UDP socket limits are too small for raw RGB-D transport."
  echo "Current: rmem_max=$DDS_RMEM_MAX, wmem_max=$DDS_WMEM_MAX"
  echo "Required: at least $DDS_REQUIRED_SOCKET_BUFFER bytes"
  echo "Run once: sudo $WS/install_aubo_socket_buffers.sh"
  exit 1
fi

echo "=========================================="
echo " RANDOM ${CUBE_COUNT}-CUBE VISUAL PICK"
echo "=========================================="

echo
echo "[1/3] Checking simulation interfaces..."

SERVICE_CARTESIAN_STATUS=UNKNOWN
SERVICE_SUCTION_STATUS=UNKNOWN
RGB_TOPIC_STATUS=UNKNOWN
DEPTH_TOPIC_STATUS=UNKNOWN
RGB_DATA_STATUS=UNKNOWN
DEPTH_DATA_STATUS=UNKNOWN
CLOCK_DATA_STATUS=UNKNOWN
CAMERA_PROBE_TIMEOUT=8

system_interfaces_ready() {
  local services topics
  services="$(ros2 service list --no-daemon 2>/dev/null || true)"
  topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"

  if grep -Fxq '/compute_cartesian_path' <<< "$services"; then
    SERVICE_CARTESIAN_STATUS=OK
  else
    SERVICE_CARTESIAN_STATUS=MISSING
  fi

  if grep -Fxq '/suction_on' <<< "$services"; then
    SERVICE_SUCTION_STATUS=OK
  else
    SERVICE_SUCTION_STATUS=MISSING
  fi

  if grep -Fxq "$AUBO_RGB_TOPIC" <<< "$topics"; then
    RGB_TOPIC_STATUS=OK
  else
    RGB_TOPIC_STATUS=MISSING
  fi

  if grep -Fxq "$AUBO_DEPTH_TOPIC" <<< "$topics"; then
    DEPTH_TOPIC_STATUS=OK
  else
    DEPTH_TOPIC_STATUS=MISSING
  fi

  active_camera_streams_ready

  [[ "$SERVICE_CARTESIAN_STATUS" == OK \
    && "$SERVICE_SUCTION_STATUS" == OK \
    && "$RGB_TOPIC_STATUS" == OK \
    && "$DEPTH_TOPIC_STATUS" == OK \
    && "$RGB_DATA_STATUS" == OK \
    && "$DEPTH_DATA_STATUS" == OK \
    && "$CLOCK_DATA_STATUS" == OK ]]
}

active_camera_streams_ready() {
  # `ros2 topic list --no-daemon` only proves that a graph name exists. Probe one real
  # sensor message from each stream so a paused/stalled Gazebo is rejected
  # before cubes are spawned and the visual queue is started. Probe sequentially:
  # launching two raw-image echo subscribers together can saturate DDS discovery /
  # receive buffers and randomly report one healthy stream as NO_DATA.

  if [[ "$RGB_TOPIC_STATUS" == OK ]]; then
    timeout "$CAMERA_PROBE_TIMEOUT" ros2 topic echo --no-daemon --once \
      --no-arr \
      --qos-reliability best_effort \
      --qos-durability volatile \
      "$AUBO_RGB_TOPIC" >/dev/null 2>&1 \
      && RGB_DATA_STATUS=OK || RGB_DATA_STATUS=NO_DATA
  else
    RGB_DATA_STATUS=TOPIC_MISSING
  fi

  if [[ "$DEPTH_TOPIC_STATUS" == OK ]]; then
    timeout "$CAMERA_PROBE_TIMEOUT" ros2 topic echo --no-daemon --once \
      --no-arr \
      --qos-reliability best_effort \
      --qos-durability volatile \
      "$AUBO_DEPTH_TOPIC" >/dev/null 2>&1 \
      && DEPTH_DATA_STATUS=OK || DEPTH_DATA_STATUS=NO_DATA
  else
    DEPTH_DATA_STATUS=TOPIC_MISSING
  fi

  if ros2 topic list --no-daemon 2>/dev/null | grep -Fxq /clock; then
    timeout "$CAMERA_PROBE_TIMEOUT" ros2 topic echo --no-daemon --once \
      --no-arr \
      --qos-reliability best_effort \
      --qos-durability volatile \
      /clock >/dev/null 2>&1 \
      && CLOCK_DATA_STATUS=OK || CLOCK_DATA_STATUS=NO_DATA
  else
    CLOCK_DATA_STATUS=TOPIC_MISSING
  fi
}

# Allow each stream enough time for DDS discovery and one real message. Image
# probes are intentionally sequential to avoid false timeouts on large RGB-D
# samples; a healthy 15 Hz camera normally completes all three immediately.
SYSTEM_READY=false
if system_interfaces_ready; then
  SYSTEM_READY=true
fi

if [[ "$SYSTEM_READY" != true ]]; then
  echo "ERROR: system interfaces or active sensor streams were not ready within ${CAMERA_PROBE_TIMEOUT}s."
  echo "Readiness details:"
  echo "  /compute_cartesian_path: $SERVICE_CARTESIAN_STATUS"
  echo "  /suction_on:            $SERVICE_SUCTION_STATUS"
  echo "  $AUBO_RGB_TOPIC: $RGB_TOPIC_STATUS / data=$RGB_DATA_STATUS"
  echo "  $AUBO_DEPTH_TOPIC: $DEPTH_TOPIC_STATUS / data=$DEPTH_DATA_STATUS"
  echo "  /clock:                 data=$CLOCK_DATA_STATUS"
  echo "Topic names can exist while Gazebo is paused; check actual data with:"
  echo "  ros2 topic hz $AUBO_RGB_TOPIC"
  echo "  ros2 topic hz $AUBO_DEPTH_TOPIC"
  echo "  ros2 topic hz /clock"
  echo "Missing interfaces can be checked with:"
  echo "  ros2 service list --no-daemon | grep -E 'compute_cartesian_path|suction_on'"
  echo "  ros2 topic list --no-daemon | grep '/camera/workcell_camera'"
  exit 1
fi

echo "System interfaces OK."

echo
echo "[2/3] Resetting and spawning ${CUBE_COUNT} random cubes..."

SPAWN_TIMEOUT=$((20 + 5 * CUBE_COUNT))
timeout "$SPAWN_TIMEOUT" python3 "$WS/spawn_random_cubes.py"

echo "Random cubes spawned."

echo
echo "[3/3] Starting one visual-pick process group..."
echo "Combined log: $PICK_LOG"
echo

set +e
PYTHONUNBUFFERED=1 ros2 launch \
  aubo_vision_grasp \
  visual_pick_system.launch.py \
  use_sim_time:=true \
  2>&1 | tee "$PICK_LOG"
PICK_STATUS=${PIPESTATUS[0]}
set -e

# ROS launch can finish cleanly after a child reports an application failure.
# Promote the two authoritative failure markers to this script's exit status.
if grep -q 'DYNAMIC PICK FAILED:' "$PICK_LOG"; then
  PICK_STATUS=1
fi
if grep -qE '(red_target_detector|visual_pick_queue).*process has died' "$PICK_LOG"; then
  PICK_STATUS=1
fi

if [ "$PICK_STATUS" -ne 0 ]; then
  echo "ERROR: visual-pick system exited with status $PICK_STATUS."
  echo "Log: $PICK_LOG"
  exit "$PICK_STATUS"
fi

echo
echo "=========================================="
echo " RANDOM PICK FINISHED"
echo "=========================================="
