#!/bin/bash


# get shell on this system
shell=$(echo $SHELL | awk -F '/' '{print $NF}')

# source ros2_ws/install/setup.bash
source ${HOME}/ROSKY2/ros2_ws/install/local_setup.${shell}

# set vehicle name
source ${HOME}/ROSKY2/setup/shell_scripts/set_vehicle_name.sh
