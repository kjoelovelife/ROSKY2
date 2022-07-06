#!/bin/bash

sudo cp ${HOME}/ROSKY2/ros2_ws/src/ominibot_car/startup/ominibot_car.rules /etc/udev/rules.d

service udev reload
sleep 2
service udev restart

