#!/bin/usr/python
#
#Copyright (c) 2021 Wei-Chih Lin
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#


import os, sys, re
from ruamel.yaml import YAML


# setup argument
shell = os.popen("echo $SHELL | awk -F '/' '{print $NF}'").readlines()[0].rstrip("\n")
home_path = f"/home/" + os.popen("echo $USERNAME | awk -F '/' '{print $NF}'").readlines()[0].rstrip("\n")
ubuntu = os.popen("grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}'").readlines()[0].rstrip("\n")

# file path
target_file = f"{home_path}/ros_menu/config.yaml"
ros_distro_file = f"{os.getcwd()}/env_variables/ros_distro.yaml"

# get project title
project = re.split('/', os.getcwd())
project = project[len(project) - 2]  # find project ROSKY2

# set function
yaml = YAML(typ="safe")


# Read YAML file.
with open(ros_distro_file) as _file:
    ros_distro = yaml.load(_file)

with open(target_file) as _file:
    target_content = yaml.load(_file)

# get the ros distro
ros1_distro = "ROS 1 " + ros_distro["ROS1"][ubuntu]
ros2_distro = "ROS 2 " + ros_distro["ROS2"][ubuntu]

# set ~/ros_menu/config.yaml
source_package = f"{home_path}/{project}/install/setup.bash"
target_content["Menu"][ros2_distro]["cmds"].append(source_package)

# write to YAML file
with open(target_file, "w") as _file:
    yaml.default_flow_style = False
    yaml.dump(target_content, _file)


