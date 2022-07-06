#!/bin/bash
#
# Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
#   This file will help you setup the environment for ROSKY2 
#   Support platform:
#     1. reComputer J1010 with Jetpack 4.6 
#        (https://www.icshop.com.tw/product-page.php?28703)
#     2. Jetson Nano Developer kit for third party Ubuntu 20.04 with Jetpack 4.3
#        (https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)
#



# Globals Parameters
RECORD_FILE=${HOME}/ROSKY2/install_script/record.txt
UBUNTU_DISTRO=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ROS2_DISTRO=foxy


#######################################
# Use apt install ros dependencies for this project
# Globals:
#   UBUNTU_DISTRO
# Arguments:
#   None
#######################################
apt_install_ros_dependencies(){
    if [ "${UBUNTU_DISTRO}" == "18.04" ]
    then
        sudo apt install -y ros-melodic-rqt-reconfigure
    else
        echo "Package rqt-reconfigure no support binary file for foxy. "
    fi
    
}

#######################################
# Use apt install build essential components
# Globals:
#   None
# Arguments:
#   None
#######################################
apt_install_dependencies(){
    sudo apt install -y python3-serial \
                        terminator \
                        byobu \
                        gedit \
                        vim \
                        isc-dhcp-server \
                        bridge-utils \
                        cmake \
                        pkg-config \
                        swig

}


#######################################
# Use pip3 install build essential components
# Globals:
#   None
# Arguments:
#   None
#######################################
pip3_install_dependencies(){
    pip3 install ruamel.yaml \
                 numpy \
                 virtualenv \
                 empy \
                 catkin_pkg \
                 lark
                         
}

#######################################
# Install Ydlidar SDK
# Globals:
#   RECORD_FILE
# Arguments:
#   project name
#######################################
ydlidar_sdk_install(){
    if [ -d "${HOME}/${1}/setup/YDLidar-SDK" ]
    then
        mkdir -p ${HOME}/${1}/setup/YDLidar-SDK/build
    else
        git clone https://github.com/YDLIDAR/YDLidar-SDK.git ${HOME}/${1}/setup
    fi 
    cd ${HOME}/${1}/setup/YDLidar-SDK/build && cmake ..
    make
    sudo make install | tee -a ${RECORD_FILE}
    echo "YDLIDAR-SDK install done!" | tee -a ${RECORD_FILE}
}


#######################################
# Add udev rules for Ydlidar and ominibot car
# Globals:
#   RECORD_FILE
# Arguments:
#   project name
#######################################
add_udev_rules(){
    echo "Setup YDLidar X4 and ominibot car." | tee -a ${RECORD_FILE}
    sudo $SHELL -c ". ${HOME}/${1}/ros2_ws/src/ydlidar_ros2_driver/startup/initenv.sh"
    sudo $SHELL -c ". ${HOME}/${1}/ros2_ws/src/ominibot_car/startup/initenv.sh"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}


#######################################
# Configure ros_menu
# Globals:
#   UBUNTU_DISTRO
#   RECORD_FILE
# Arguments:
#   None
#######################################
config_ros_menu(){
    if [ -d "${HOME}/ros_menu" ]
    then
        if [ -f "${HOME}/ros_menu/config.yaml" ]
        then
            sudo rm -rf ${HOME}/ros_menu/config.yaml
        fi
    else
        git clone https://github.com/adlink-ros/ros_menu.git ~/ros_menu
    fi

    cd $HOME/ros_menu && echo "n" | ./install.sh | tee -a ${RECORD_FILE}
    sed -i "s:dashing:foxy:g" ${HOME}/ros_menu/config.yaml

    # Congigure virtualenv for Python3 from ubuntu 18.04
    if [ "${UBUNTU_DISTRO}" == "18.04" ]
    then
        if [ -d "${HOME}/virtualenv" ]
        then
            virtualenv -p python3 ${HOME}/virtualenv/python3 | tee -a ${RECORD_FILE} 
            sed -i "24 a \ \ \ \ \ \ -\ source\ ${HOME}/virtualenv/python3/bin/activate\n\ \ \ \ \ \ -\ export OPENBLAS_CORETYPE=ARMV8" \
              ${HOME}/ros_menu/config.yaml
        fi
    fi


}

#######################################
# Modified syntax in setup.cfg with in all package.
# Globals:
#   None
# Arguments:
#   project name
#######################################
modified_sytax_in_setup_cfg(){
    sed -i "s:script\-dir:script_dir:g;s:install\-scripts:install_scripts:g" $(find ${HOME}/${1}/ros2_ws -iname "setup.cfg" -type f)
}


#######################################
# main function
# Globals:
#   None
# Arguments:
#   None
#######################################
main(){   
    touch $RECORD_FILE
    echo -e "\n====== Start $(date) ======\n" >> $RECORD_FILE

    apt_install_ros_dependencies
    apt_install_dependencies
    pip3_install_dependencies
    ydlidar_sdk_install ROSKY2
    add_udev_rules ROSKY2
    config_ros_menu

    echo -e "\n====== End $(date) ======\n" >> $RECORD_FILE

    exit 0
}


# start the progress
main




