#!/bin/bash

sudo cp ominibot_car.rules /etc/udev/rules.d

service udev reload
sleep 2
service udev restart

