SHELL := /bin/bash

SOURCE_DIR = src/ping_pong_measurer/src

starter: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer starter

run_ping: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer ping --node-counts 10 --payload-bytes 10 --measurement-times 100

run_pong: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer pong --node-counts 10

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
