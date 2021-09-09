#!/usr/bin/env bash

path_kinematics="ROSKY/catkin_ws/src/rosky_base/config/baseline/calibration/kinematics"
path_keyboard_mapper_node="ROSKY/catkin_ws/src/rosky_base/config/baseline/keyboard_mapper/keyboard_mapper_node"
path_camera_intrinsic="ROSKY/catkin_ws/src/rosky_base/config/baseline/calibration/camera_intrinsic"
path_camera_extrinsic="ROSKY/catkin_ws/src/rosky_base/config/baseline/calibration/camera_extrinsic"
path_camera_info="ROSKY/catkin_ws/src/jetson_camera/camera_info/cam_info_reader_node"
path_decoder_node="ROSKY/catkin_ws/src/jetson_camera/camera_info/decoder_node"
path_line_detector="ROSKY/catkin_ws/src/rosky_base/config/baseline/line_detector/line_detector_node"
path_lane_controller="ROSKY/catkin_ws/src/rosky_base/config/baseline/lane_control/lane_controller_node"
default="default.yaml"

echo "Setting up parameter files with project ROSKY... "

if [ $# -gt 0 ]; then
        file_name=$1
	echo "You provide a hostname. The files name are: $file_name.yaml"
else
        file_name=$HOSTNAME
	echo "No hostname provided. Files name Use $HOSTNAME."

fi

#==== Set up kinematics ==== 
echo ""
echo "Setup parameter kinematics"
if test -e ~/$path_kinematics/$file_name".yaml"; then
    echo -n "You already have the parameter about kinematics, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_kinematics/$default ~/$path_kinematics/$file_name".yaml"
        echo "Reset param kinematics."
    else
        echo "Skip set param kinematics."
    fi
else
    cp ~/$path_kinematics/$default ~/$path_kinematics/$file_name".yaml"
    echo "Add param kinematics with name \"$file_name\""
fi


#==== Set up keyboard_mapper_node ====
echo ""
echo "Setup parameter keyboard_mapper_node"
if test -e ~/$path_keyboard_mapper_node/$file_name".yaml"; then
    echo -n "You already have the parameter about keyboard_mapper_node, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_keyboard_mapper_node/$default ~/$path_keyboard_mapper_node/$file_name".yaml"
        echo "Reset param keyboard_mapper_node."
    else
        echo "Skip set param keyboard_mapper_node."
    fi
else
    cp ~/$path_keyboard_mapper_node/$default ~/$path_keyboard_mapper_node/$file_name".yaml"
    echo "Add param keyboard_mapper_node with name \"$file_name\""
fi

#==== Set up camera_intrinsic ====
echo ""
echo "Setup parameter camera_intrinsic"
if test -e ~/$path_camera_intrinsic/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_intrinsic, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_intrinsic/$default ~/$path_camera_intrinsic/$file_name".yaml"
        echo "Reset param camera_intrinsic."
    else
        echo "Skip set param camera_intrinsic."
    fi
else
    cp ~/$path_camera_intrinsic/$default ~/$path_camera_intrinsic/$file_name".yaml"
    echo "Add param camera_intrinsic with name \"$file_name\""
fi

#==== Set up camera_extrinsic ====
echo ""
echo "Setup parameter camera_extrinsic"
if test -e ~/$path_camera_extrinsic/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_extrinsic, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_extrinsic/$default ~/$path_camera_extrinsic/$file_name".yaml"
        echo "Reset param camera_extrinsic."
    else
        echo "Skip set param camera_extrinsic."
    fi
else
    cp ~/$path_camera_extrinsic/$default ~/$path_camera_extrinsic/$file_name".yaml"
    echo "Add param camera_extrinsic with name \"$file_name\""
fi

#==== Set up camera_extrinsic ====
echo ""
echo "Setup parameter camera_info"
if test -e ~/$path_camera_info/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_info, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_info/$default ~/$path_camera_info/$file_name".yaml"
        echo "Reset param camera_info."
    else
        echo "Skip set param camera_info."
    fi
else
    cp ~/$path_camera_info/$default ~/$path_camera_info/$file_name".yaml"
    echo "Add param camera_info with name \"$file_name\""
fi

#==== Set up decoder_node ====
echo ""
echo "Setup parameter decoder_node"
if test -e ~/$path_decoder_node/$file_name".yaml"; then
    echo -n "You already have the parameter about decoder_node, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_decoder_node/$default ~/$path_decoder_node/$file_name".yaml"
        echo "Reset param decoder_node."
    else
        echo "Skip set param decoder_node."
    fi
else
    cp ~/$path_decoder_node/$default ~/$path_decoder_node/$file_name".yaml"
    echo "Add param decoder_node with name \"$file_name\""
fi

#==== Set up line_detector ====
echo ""
echo "Setup parameter line_detector"
if test -e ~/$path_line_detector/$file_name".yaml"; then
    echo -n "You already have the parameter about line_detector, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_line_detector/$default ~/$path_line_detector/$file_name".yaml"
        echo "Reset param line_detector."
    else
        echo "Skip set param line_detector."
    fi
else
    cp ~/$path_line_detector/$default ~/$path_line_detector/$file_name".yaml"
    echo "Add param line_detector with name \"$file_name\""
fi

#==== Set up lane_controller ====
echo ""
echo "Setup parameter lane_controller"
if test -e ~/$path_lane_controller/$file_name".yaml"; then
    echo -n "You already have the parameter about lane_controller, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_lane_controller/$default ~/$path_lane_controller/$file_name".yaml"
        echo "Reset param lane_controller."
    else
        echo "Skip set param lane_controller."
    fi
else
    cp ~/$path_lane_controller/$default ~/$path_lane_controller/$file_name".yaml"
    echo "Add param lane_controller with name \"$file_name\""
fi

#==== Set up rviz ====
if test -d ~/.rviz; then
    echo ""
else
    echo "Make /home/$USER/.rviz"
    mkdir /home/$USER/.rviz
fi
echo "Copy the \"/home/$USERNAME/ROSKY/catkin_ws/src/rosky_slam/rviz/default.rviz\" to /home/$USER/.rviz "
cp /home/$USER/ROSKY/catkin_ws/src/rosky_slam/rviz/default.rviz /home/$USER/.rviz/default.rviz



echo ""
echo "Finish setting up."
echo ""
