#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$0")/environnement.sh"
mission="${1:-01}"
case "$mission" in 01|02|03|04) ;; *) exit 1;; esac
mkdir -p "$WS_DIR/essais"
exec ros2 bag record -o "$WS_DIR/essais/m${mission}-$(date +%Y%m%d-%H%M%S)"  "$SCAN_TOPIC" "$ODOM_TOPIC" "$IMAGE_TOPIC" "$CMD_TOPIC"  "/urmc/m$mission/status" "/urmc/test/m$mission/cmd_vel"
