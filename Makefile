SHELL := /bin/bash

SOURCE_DIR = src/ping_pong_measurer/src

run_starter: build
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer starter

run_ping: build
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer ping --pong-node-count 1 --payload-bytes 256 --measurement-times 100 --pub single --sub single --enable-os-info-measuring

run_pong: build
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer pong --node-counts 1 --pub single --sub single

run_talker:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer talker

run_listener:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer listener

build: $(SOURCE_DIR)/ping.cpp $(SOURCE_DIR)/pong.cpp $(SOURCE_DIR)/starter.cpp
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --packages-select ping_pong_measurer

.PHONY: run_starter run_ping run_pong run_talker run_listener build
