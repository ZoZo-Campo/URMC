#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")/.."
[[ "$(uname -s)" == Linux && -f /opt/ros/humble/setup.bash ]] || { echo 'Ce kit exige Linux et ROS 2 Humble. Envoyer l’inventaire si la version diffère.'; exit 1; }
source /opt/ros/humble/setup.bash
for tool in colcon rosdep; do command -v "$tool" >/dev/null || { echo "$tool absent : compléter l’installation avec le responsable du robot."; exit 1; }; done
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
printf '
Compilation terminée. Compléter kit_robot/profil_robot.sh avant les essais.
'
