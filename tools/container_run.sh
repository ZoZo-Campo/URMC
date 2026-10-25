#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
mkdir -p /ws
cp -aL /project/src /project/tests /project/tools /project/kit_robot /ws/
cd /ws
colcon build --symlink-install > /tmp/urmc-build.log 2>&1 || { cat /tmp/urmc-build.log; exit 1; }
source install/setup.bash
if [[ "$1" == test ]]; then
  python3 -m pytest -q -p no:cacheprovider tests
  python3 tools/controls_ros.py
  python3 tools/integration_ros.py
  python3 tools/faults_ros.py
  python3 tools/evaluate_lidar.py
  python3 tools/evaluate_line.py
  python3 tools/test_line_ros.py
else
  mission="$1"
  case "$mission" in 01|02|03|04|05) ;; *) echo 'Mission invalide'; exit 1;; esac
  exec ros2 launch "urmc_mission_$mission" sim.launch.py dashboard:=true
fi
