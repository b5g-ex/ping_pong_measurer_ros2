SHELL := /bin/bash

SOURCE_DIR = src/ping_pong_measurer/src

run_ping: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer ping

run_pong:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer pong

run_talker:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer talker

run_listener:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer listener

build: $(SOURCE_DIR)/ping.cpp $(SOURCE_DIR)/pong.cpp
	source /opt/ros/foxy/setup.bash && \
	colcon build --packages-select ping_pong_measurer

.PHONY: build run_ping run_pong run_talker run_listener
