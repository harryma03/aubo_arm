#!/usr/bin/env bash
# Shared workspace discovery and environment setup. Safe to source repeatedly.
WS="${AUBO_GRASP_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
export AUBO_GRASP_WS="$WS"

aubo_source_setup() {
  # ROS-generated setup scripts may reference unset variables.
  local restore_nounset=false
  case "$-" in *u*) restore_nounset=true; set +u ;; esac
  local status=0
  source "$1" || status=$?
  if "$restore_nounset"; then set -u; fi
  return "$status"
}

aubo_load_config() {
  export AUBO_CONFIG_FILE="${AUBO_CONFIG_FILE:-$WS/src/aubo_workcell_description/config/workcell_layout.env}"
  export AUBO_CYCLONEDDS_CONFIG="${AUBO_CYCLONEDDS_CONFIG:-$WS/src/aubo_workcell_description/config/cyclonedds.xml}"
  local file
  for file in "$AUBO_CONFIG_FILE" "$AUBO_CYCLONEDDS_CONFIG"; do
    if [[ ! -r "$file" ]]; then
      echo "ERROR: Configuration not readable: $file" >&2
      return 1
    fi
  done
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$AUBO_CYCLONEDDS_CONFIG}"
  local restore_allexport=false
  case "$-" in *a*) ;; *) restore_allexport=true; set -a ;; esac
  local status=0
  source "$AUBO_CONFIG_FILE" || status=$?
  if "$restore_allexport"; then set +a; fi
  return "$status"
}

aubo_load_ros() {
  conda deactivate 2>/dev/null || true
  local setup="/opt/ros/${AUBO_ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -r "$setup" ]]; then
    echo "ERROR: ROS setup not found: $setup" >&2
    return 1
  fi
  aubo_source_setup "$setup"
}

aubo_load_runtime() {
  aubo_load_ros || return
  if [[ ! -r "$WS/install/setup.bash" ]]; then
    echo "ERROR: Workspace not built. Run: $WS/aubo.sh build" >&2
    return 1
  fi
  aubo_source_setup "$WS/install/setup.bash" || return
  aubo_load_config
}
