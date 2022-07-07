#!/bin/bash

# get shell on this system
shell=$(echo $SHELL | awk -F '/' '{print $NF}')

# show information
echo -e "\nSetup operating environment for ROSKY2"


# source ros2_ws/install/setup.bash
source ${HOME}/ROSKY2/ros2_ws/install/setup.${shell}

# set vehicle name
source ${HOME}/ROSKY2/setup/shell_scripts/set_vehicle_name.sh

# show information
echo -e "Now you can start using ROSKY2.\n"
