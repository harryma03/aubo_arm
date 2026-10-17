#!/bin/bash

# Keep the pick-run detector alive, but allow only one automatic restart.
# The parent run_random_pick.sh terminates this supervisor; its trap then
# terminates the detector child as well.

set -u

WS="$1"
LOG_FILE="$2"
PID_FILE="$3"
MAX_RESTARTS="${AUBO_VISION_MAX_RESTARTS:-1}"

cd "$WS"

restart_count=0
detector_pid=""

cleanup() {
    trap - INT TERM
    if [ -n "$detector_pid" ] && kill -0 "$detector_pid" 2>/dev/null; then
        kill -TERM "$detector_pid" 2>/dev/null || true
        wait "$detector_pid" 2>/dev/null || true
    fi
    rm -f "$PID_FILE"
    exit 143
}

trap cleanup INT TERM

while true; do
    echo "[VISION SUPERVISOR] starting detector; restart=$restart_count" \
        >> "$LOG_FILE"

    # Keep the child PID so the supervisor trap can terminate ros2-run cleanly.
    PYTHONUNBUFFERED=1 ros2 run \
        aubo_vision_grasp \
        red_target_detector \
        --ros-args \
        -p use_sim_time:=true \
        -r __node:=red_target_detector_pick \
        >> "$LOG_FILE" 2>&1 &

    detector_pid=$!
    echo "$detector_pid" > "$PID_FILE"

    set +e
    wait "$detector_pid"
    detector_status=$?
    set -e

    rm -f "$PID_FILE"

    echo "[VISION SUPERVISOR] detector exited with status $detector_status" \
        >> "$LOG_FILE"

    if [ "$restart_count" -ge "$MAX_RESTARTS" ]; then
        echo "[VISION SUPERVISOR] restart limit reached; stopping" \
            >> "$LOG_FILE"
        exit "$detector_status"
    fi

    restart_count=$((restart_count + 1))
    sleep 1
done
