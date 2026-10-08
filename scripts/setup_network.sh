#!/usr/bin/env bash

# Install the one-time host configuration required by raw RGB-D DDS traffic.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/lib/env.sh"
SOURCE_FILE="$WS/src/aubo_workcell_description/config/99-aubo-rgbd-socket-buffers.conf"
TARGET_FILE="/etc/sysctl.d/99-aubo-rgbd-socket-buffers.conf"

if (( EUID != 0 )); then
  echo "ERROR: root permission is required to update Linux socket limits." >&2
  echo "Run: sudo $WS/aubo.sh setup-network" >&2
  exit 1
fi

if [[ ! -r "$SOURCE_FILE" ]]; then
  echo "ERROR: socket-buffer configuration is missing: $SOURCE_FILE" >&2
  exit 1
fi

install -m 0644 "$SOURCE_FILE" "$TARGET_FILE"
sysctl -p "$TARGET_FILE"

echo
echo "AUBO RGB-D socket buffers installed."
echo "Configuration: $TARGET_FILE"
