#!/bin/bash
#
#Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
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

# arguments
ubuntu_distro=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')

# function
apt_install_ros2(){
    '''Use apt install ros2 dependencies for this project
    Args:
      $1: project name
    '''
    distro=$1
    sudo apt install -y ros-$distro-rqt-reconfigure \
                        cmake \
                        pkg-config \
                        swig
}

apt_install_dependencies(){
    sudo apt install -y python3-serial \
                        terminator \
                        byobu \
                        gedit \
                        vim \
                        isc-dhcp-server \
                        bridge-utils

}

pip3_install_dependencies(){
    sudo -H pip3 install ruamel.yaml \
                         numpy
}


ydlidar_sdk_install(){
    '''install Ydlidar SDK 
    Args:
      $1: project name
    '''
    project=$1
    cd ${HOME}/${project}/setup && git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
    echo "YDLIDAR-SDK install done!"
    cd ${HOME}
}

config_ros_menu(){
    '''Insert command in ~/ros_menu/config.yaml
    Args:
      $1: project name
    '''
    project=$1
    cd ${HOME}/${project}/setup/python_scripts && python3 config.py
    cd ${HOME}/${project}

}

add_udev_rules(){
    '''add udev rules for Ydlidar and ominibot car 
    Args:
      $1: project name
    '''
    project=$1
    echo "Setup YDLidar X4 and ominibot car."
    sudo $SHELL ${HOME}/${project}/ros2_ws/src/ydlidar_ros2_deiver/startup/initenv.sh
    sudo $SHELL ${HOME}/${project}/ros2_ws/src/ominibotcar/startup/initenv.sh
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}


# Install dependencies
apt_install_ros2 foxy
apt_install_dependencies
pip3_install_dependencies
ydlidar_sdk_install ROSKY2
config_ros_menu ROSKY2
add_udev_rules ROSKY2

# change directory to ROSKY2
cd ${HOME}/ROSKY2 





