SHELL := /bin/bash

run_talker:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer talker

run_listener:
	source /opt/ros/foxy/setup.bash && \
	source install/setup.bash && \
	ros2 run ping_pong_measurer listener

build:
	source /opt/ros/foxy/setup.bash && \
	colcon build --packages-select ping_pong_measurer
