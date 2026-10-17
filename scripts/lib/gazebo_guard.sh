#!/usr/bin/env bash
# Read-only guards: never kill a process merely because it owns a port.
gazebo_check_port() {
  local port listeners
  port="$(python3 -c '
import os
from urllib.parse import urlparse
uri = os.environ.get("GAZEBO_MASTER_URI", "http://localhost:11345")
try:
    parsed = urlparse(uri)
    if parsed.scheme != "http" or not parsed.hostname or not parsed.port:
        raise ValueError("expected http://host:port")
    print(parsed.port)
except ValueError as exc:
    raise SystemExit("ERROR: Invalid GAZEBO_MASTER_URI: " + str(exc))
')" || return 1
  listeners="$(ss -H -ltnp "sport = :$port")" || {
    echo "ERROR: Cannot check Gazebo port (ss is required)." >&2
    return 1
  }
  if [[ -n "$listeners" ]]; then
    echo "ERROR: Gazebo port $port is already occupied:" >&2
    echo "$listeners" >&2
    echo "Confirm the owner's identity before stopping it; no automatic kill." >&2
    return 1
  fi
}

gazebo_check_alive() {
  if ! kill -0 "$GAZEBO_LAUNCH_PID" 2>/dev/null ||
     grep -Eq '\[ERROR\].*\[gzserver-[0-9]+\].*process has died|\[INFO\].*\[gzserver-[0-9]+\].*process has finished' "$LOG/gazebo.log"; then
    echo "ERROR: Gazebo server exited; aborting startup. See $LOG/gazebo.log" >&2
    return 1
  fi
}

# ros2 launch can remain alive after gzserver dies. Also require its own server,
# so a stale ROS discovery entry alone cannot make startup appear successful.
gazebo_has_server() {
  pgrep -P "$GAZEBO_LAUNCH_PID" -x gzserver >/dev/null
}

# Capture and stop descendants while their launch parent still exists.
gazebo_stop_launch_tree() {
  local parent="$1" child
  [[ "$parent" != "$$" ]] || return 0
  while IFS= read -r child; do
    gazebo_stop_launch_tree "$child"
  done < <(pgrep -P "$parent" 2>/dev/null || true)
  kill -TERM "$parent" 2>/dev/null || true
}
