#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cp -aL /project/src /project/kit_robot /project/tools /ws/
cd /ws
colcon build --symlink-install --packages-select urmc_core urmc_mission_04 >/tmp/urmc-slam-build.log 2>&1
source install/setup.bash
export URMC_LOCALHOST_ONLY=1 ROS_LOCALHOST_ONLY=1 URMC_ROBOT_DOMAIN=99 URMC_ROBOT_ID=0 URMC_SETTINGS_FILE=/ws/session_robot11/reglages.json
mkdir -p session_robot11
python3 tools/sim_enclosure_slam.py --ros-args -r /tf:=/urmc_sim/m04_slam/tf -r /tf_static:=/urmc_sim/m04_slam/tf_static &
sim_pid=$!
trap 'kill -INT "$sim_pid" 2>/dev/null || true' EXIT
python3 kit_robot/mission_enceinte.py --simulation --ros-args -r /tf:=/urmc_sim/m04_slam/tf -r /tf_static:=/urmc_sim/m04_slam/tf_static
