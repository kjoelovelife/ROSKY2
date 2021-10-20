# Simple ROS2 example for ROSKY2 - wall follower
In this example, we'll use ROS2 and let ROSKY2 do:
1. find wall automatically with lidar
2. try to near the wall and record odometery at the same time
3. after nearing the wall, ROSKY2 will go along with the wall 

## Hardware you need to prepare before following the wall
For observing clearly, we need an enclosed space about 180(cm)x180(cm), and the walls should be more higher than lidar on the top of ROSKY2 about 13 cm.

<img src="https://i.imgur.com/075W1Rx.jpg" width="300"/>

And then put ROSKY2 in the enclosed space anywhere you want 

<img src="https://i.imgur.com/ao37Ged.jpg" width="300"/>

Also, prepare a WI-FI router and let ROSKY2 connect the WI-FI hotspot.

If you don't know how to let ROSKY2 connect WI-FI hotspot without display, please visit [Setup Jetson nano without display](https://hackmd.io/@weichih-lin/Jetson_nano_withuout_display).

OK! Now we have already to use the example - wall follower wtih ROSKY2.

## 1. Steps for using wall_follower
Make sure you have done the step **[Hardware you need to prepare before following the wall](https://hackmd.io/@weichih-lin/ROSKY2_wall_follower#Hardware-you-need-to-prepare-before-following-the-wall)** and **[build project ROSKY2 workspace](https://hackmd.io/@weichih-lin/ROSKY2_setup_environment#4-build-project-ROSKY2-workspace)** before running the steps below.

Before start following wall, let me show you a useful tool to help you control the ROSKY2 - Terminator. If you have installed ROSKY2 dependencies through [rosky2_dependencies.sh](https://github.com/kjoelovelife/ROSKY2/blob/main/install_script/rosky2_dependencies.sh) then you can type ``` terminator ``` in terminal to start Terminator.

Terminator is an alternative terminal for Linux that comes with a little additional features and functionality that you want to find in the default terminal application. In this example, we will use basic function - multiple terminals in one window.

![Terminator](https://i.imgur.com/sw1CjRj.png)

And there are some keys usually use:

| Keys                   | Function                                                  |
| -----------------------| --------------------------------------------------------- |
| [ctrl] + [shift] + [e] | start new terminator beside current terminator(horizontal)|
| [ctrl] + [shift] + [o] | start new terminator under current terminator(vertical)   |
| [ctrl] + [shift] + [w] | close current terminator                                  |

### 1-1. bringup the ROSKY2
We need to start the communication between Jetson nano, lidar and motor control board. 
Please type the code below in the terminator to brinup the ROSKY2:

```bash=
ros2 launch rosky2_bringup bringup.launch.py rf2o:=true 
```
Great! In this case, we will use lidar to publish odom, so remember to set LaunchConfiguration "rf2o" to true.

And if everything is fine, then you can see the information on the terminator and lidar will start rotating.

### 1-2. start the main launch file
The main launch file consist of [wall_following.launch.py](https://github.com/kjoelovelife/ROSKY2/blob/main/ros2_ws/src/wall_follower/launch/wall_following.launch.py), [find_wall.launch.py](https://github.com/kjoelovelife/ROSKY2/blob/main/ros2_ws/src/wall_follower/launch/find_wall.launch.py) and [record_odometry_action_server.launch.py](https://github.com/kjoelovelife/ROSKY2/blob/main/ros2_ws/src/wall_follower/launch/record_odometry_action_server.launch.py). Please type the code below in the terminator to start the mission:

```bash=
ros2 launch wall_follower main.launch.py
```

And if everything is fine, you can see the information include how far ROSKY2 go and all recorded odometery before following the wall on the terminator.

Please visit it to wacth the full operating video: [ROSKY2 and wall_follower](https://youtu.be/GTSCI3VpJeo)

## More detail you may want to know


| Name                        | Description        |
| --------------------------- | ------------------ |
| [1. ROS2 and Topic]()       | to be continued... |
| [2. ROS2 and Service]()     | to be continued... |
| [3. ROS2 and Action]()      | to be continued... |
| [4. ROS2 and Interface]()   | to be continued... |




