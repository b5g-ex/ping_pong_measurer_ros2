SHELL := /bin/bash

SOURCE_DIR = src/ping_pong_measurer/src

os_info: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer os_info

run_starter: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer starter

run_ping: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer ping --pong-node-count 10 --payload-bytes 10 --measurement-times 100 --pub single --sub single

run_pong: build
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer pong --node-counts 10 --pub single --sub single

run_talker:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer talker

run_listener:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer listener

build: $(SOURCE_DIR)/ping.cpp $(SOURCE_DIR)/pong.cpp $(SOURCE_DIR)/starter.cpp $(SOURCE_DIR)/os_info.cpp
	source /opt/ros/foxy/setup.bash && \
	colcon build --packages-select ping_pong_measurer

.PHONY: build run_ping run_pong run_talker run_listener
