#!/bin/bash

## get the argument
if [ $# -gt 0 ]; then
    ROS2_distro="$1"
else
    ROS2_distro="dashing"
fi

## set the argument
project="ROSKY2"

## clear ROS env var for resolving warning ##
unset ros_option
unset ROS_DISTRO
unset ROS_HOSTNAME

## configure ros2
source /opt/ros/$ROS2_distro/setup.bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=$ROS2_distro

## setup ROS_DOMAIN_ID
source ~/$project/setup/set_domain_id.sh 

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

