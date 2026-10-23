#!/usr/bin/env bash
# À charger avec source par les autres scripts.
KIT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname -- "$KIT_DIR")"
source "$KIT_DIR/profil_robot.sh"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || { echo 'Compléter ROS_DOMAIN_ID dans kit_robot/profil_robot.sh.' >&2; return 1; }
[[ -f /opt/ros/humble/setup.bash && -f "$WS_DIR/install/setup.bash" ]] || { echo 'ROS Humble ou workspace compilé absent.' >&2; return 1; }
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
