#!/usr/bin/env bash

# Single shell entry point for every workcell/grasp parameter.
# Source this file after ROS setup.  It exports all values declared by the
# configuration file so Python nodes and child launch processes see the same
# environment without duplicating defaults in multiple startup scripts.

if [[ -n "${AUBO_GRASP_WS:-}" ]]; then
  _aubo_ws="$AUBO_GRASP_WS"
else
  _aubo_loader_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _aubo_ws="$_aubo_loader_dir"
fi

export AUBO_GRASP_WS="$_aubo_ws"
export AUBO_CONFIG_FILE="${AUBO_CONFIG_FILE:-$_aubo_ws/src/aubo_workcell_description/config/workcell_layout.env}"
export AUBO_CYCLONEDDS_CONFIG="${AUBO_CYCLONEDDS_CONFIG:-$_aubo_ws/src/aubo_workcell_description/config/cyclonedds.xml}"

if [[ ! -r "$AUBO_CONFIG_FILE" ]]; then
  echo "ERROR: AUBO configuration is not readable: $AUBO_CONFIG_FILE" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ ! -r "$AUBO_CYCLONEDDS_CONFIG" ]]; then
  echo "ERROR: CycloneDDS configuration is not readable: $AUBO_CYCLONEDDS_CONFIG" >&2
  return 1 2>/dev/null || exit 1
fi

# Keep an explicitly supplied DDS configuration, otherwise use the workcell's
# high-bandwidth profile for raw 640x480 RGB-D images.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$AUBO_CYCLONEDDS_CONFIG}"
# The simulation is local by default. Override with
# AUBO_ROS_LOCALHOST_ONLY=0 only when ROS nodes must run on another host.
export ROS_LOCALHOST_ONLY="${AUBO_ROS_LOCALHOST_ONLY:-1}"


case "$-" in
  *a*) _aubo_restore_allexport=false ;;
  *)   _aubo_restore_allexport=true; set -a ;;
esac

# shellcheck source=/dev/null
source "$AUBO_CONFIG_FILE"

if "$_aubo_restore_allexport"; then
  set +a
fi

unset _aubo_restore_allexport _aubo_loader_dir _aubo_ws
