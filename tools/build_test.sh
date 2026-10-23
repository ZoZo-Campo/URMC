#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
python3 -m pytest -q -p no:cacheprovider tests
