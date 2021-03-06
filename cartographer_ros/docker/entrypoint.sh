#!/bin/bash
set -e

echo "Starting cartographer_ros with ROS_MASTER_URI=$ROS_MASTER_URI"

source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/cartographer_ros/setup.bash

if [ "$CARTO_MODE" = "localization" ]; then
     wget -O /current_map_state.pbstream "$CARTO_MAP"
     roslaunch cartographer_ros ${MODEL}_localization.launch load_state_filename:=/current_map_state.pbstream
else
     roslaunch cartographer_ros ${MODEL}_slam.launch
fi
