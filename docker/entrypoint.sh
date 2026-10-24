#!/bin/bash
# Source ROS + TurtleBot3, build this workspace once if needed, then run the command.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash

if [ -d /ws/src/autonomous_nav ] && [ ! -f /ws/install/setup.bash ]; then
    echo "[entrypoint] Building autonomous_nav..."
    (cd /ws && colcon build --packages-select autonomous_nav --symlink-install)
fi
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

exec "$@"
