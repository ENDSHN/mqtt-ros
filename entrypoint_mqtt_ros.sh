#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch --wait mqtt_ros mqtt_ros.launch mqtt_broker_address:=${MQTT_BROKER_ADDRESS} robot_name:=${ROBOT_NAME} config_file:=${CONFIG_FILE}