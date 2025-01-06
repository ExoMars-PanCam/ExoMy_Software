#!/bin/bash
set -e

if [[ $1 == "config" ]]
then
	cd "/root/exomy_ws/src/exomy/scripts"
	bash
elif [[ $1 == "autostart" ]]
then
	cd "/root/exomy_ws"
	apt update
	source "/opt/ros/humble/setup.bash"
	rosdep install --from-paths src --ignore-src --skip-keys=libcamera -y
	colcon build --symlink-install
	source "/root/exomy_ws/install/setup.bash"
	
	cd "/root/exomy_ws/src/exomy/gui"

	# node node_modules/ros2-web-bridge/bin/rosbridge.js &
	# http-server -p 8000 &

	ros2 launch exomy exomy.launch.py
	# Sleep is needed to first print output and yjem start bash
	sleep 1
	
	cd "/root/exomy_ws"
	bash
elif [[ $1 == "devel" ]]
then
	cd "/root/exomy_ws"
	apt update
	source "/opt/ros/humble/setup.bash"
	rosdep install --from-paths src --ignore-src --skip-keys=libcamera -y
	colcon build --symlink-install
	source "/root/exomy_ws/install/setup.bash"
	
	bash
else
	bash
fi
