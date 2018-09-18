#!/bin/bash
set -e

echo "Starting cartographer_ros with ROS_MASTER_URI=$ROS_MASTER_URI"

source /opt/ros/kinetic/setup.bash
source /opt/cartographer_ros/setup.bash

if [ "$CARTO_MODE" = "localization" ]; then
     wget -O /current_map_state.pbstream "$CARTO_MAP"
     roslaunch cartographer_ros turtlebot${MODEL_NUM}_localization.launch load_state_filename:=/current_map_state.pbstream
else
     roslaunch cartographer_ros turtlebot${MODEL_NUM}_slam.launch
fi
