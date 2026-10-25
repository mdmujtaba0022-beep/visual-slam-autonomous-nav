.PHONY: build sim slam nav explore full map-save test clean

# `source` is a bash builtin; make's default /bin/sh (dash) doesn't have it
SHELL := /bin/bash

build:
	source /opt/ros/jazzy/setup.bash && cd $(CURDIR) && colcon build --packages-select autonomous_nav --symlink-install

sim:
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

slam:
	ros2 launch autonomous_nav slam_mapping.launch.py

nav:
	ros2 launch autonomous_nav navigation.launch.py

explore:
	ros2 launch autonomous_nav autonomous_explore.launch.py

full:
	ros2 launch autonomous_nav full_stack.launch.py

map-save:
	mkdir -p ./src/autonomous_nav/maps
	ros2 run nav2_map_server map_saver_cli -f ./src/autonomous_nav/maps/warehouse

test:
	source /opt/ros/jazzy/setup.bash && cd $(CURDIR) && colcon test --packages-select autonomous_nav && colcon test-result --verbose

clean:
	rm -rf build/ install/ log/
