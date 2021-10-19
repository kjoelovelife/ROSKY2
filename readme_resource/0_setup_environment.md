# Setup operating environment with ROSKY2
This file will tell you how to setup the Project "ROSKY2" with Jetson nano developer kit.



## Hardware you need to prepare before writing image
To prepare your microSD card, you’ll need a computer with Internet connection and the ability to read and write SD cards, either via a built-in SD card slot or adapter. Notice space of the MicroSD card need **bigger than 32 GB**.

## 1. Write Image to the microSD Card
### 1-1. Install balenaEtcher
I suggest using balenaEtcher to write the image with Microsoft windows system. Please visit the official website to download the software: [balenaEtcher official website](https://www.balena.io/etcher/) 

![download balenaEtcher](https://i.imgur.com/exyaUNy.png)


Please click the "Download for Windows(x86|x64)" button to download and then run the file to install. If you can install smoothly then you can see the icon "balenaEtcher" in the desktop.

### 1-2. Download the image
Nvidia don't offer ubuntu 20.04 image for the Jetson nano now(2021/10/18). We can use the third-party image to write to the microSD Card. Please visit this webiste to download and unzip file: [Xubuntu 20.04 Focal Fossa L4T R32.3.1 - Custom Image for the Jetson Nano](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)

![third party image](https://i.imgur.com/Kwv98H4.png)


### 1-3. Start writing image!
Ok! Now we have already prepared to write image to the microSD Card! Please double click icon "balenaEtcher" on your desktop to run the software.

![start balenaEtcher](https://i.imgur.com/DQmpEpf.png)

If you can see the image above on your desktop, then you can follow the steps below to write image:
> 1. click "Flash" from file and select your image. If you don't rename the image file, you should select "Xubuntu-20.04-l4t-r32.3.1.img" 
> 2. If you use adapter, please insert it into your computer and then click "Select target" to select your microSD card. 

Great! Now everything is done, you can see the similar image below:

![ready_to_write](https://i.imgur.com/YWHGbUP.png)


Please click the blue button "Flash!", and then you will need some time to flash the image. If the balenaEtcher finish flashing normally, you can see the similar image below:

![finish_flashing](https://i.imgur.com/n8jrhNj.png)


Now you can take out the microSD card from your computer.

## 2. Install Ubuntu 20.04 on ROSKY  
In this step,  we have to install Ubuntu 20.04 through the microSD card. Please insert the microSD card into the Jetson nano. After that, you can power on Jetson nano and follow install steps to install Ubuntu 20.04.

## 3. Install ROS1 and ROS2 through ros_menu and dependencies for project ROSKY2
we can use the [ros_menu](https://github.com/Adlink-ROS/ros_menu) provided by [Adlink](https://www.adlinktech.com/en/index.aspx) to install ROS1, ROS2 and package ros-foxy-ros1-bridge. Please followe these steps below to install.

### 3-1. open the terminator
Please press **[ctrl] + [alt] + [t]** on desktop to open the terminal

![open terminal](https://i.imgur.com/MiGvy0o.png)

### 3-2. type command to install ros_menu automatically
Please type command below in the terminal

```bash=
source ~/ROSKY2/install_script/install_adlink_ros_menu.sh
```

After that, please follow steps show on the terminal to install ros_menu. 

### 3-3. install dependenices for project ROSKY2
After finish install ros_menu, we need to resource **~/.bashrc** to update the terminal you use now. Please type command below to update:

```bash=
source ~/.bashrc
```

After update, you can see some selections which environment you want to use in terminal, please type **2** to choose ROS2:

![choose_ROS2](https://i.imgur.com/WKRGTA8.png)

Great! Now we can use the script [rosky2_dependencies.sh](https://github.com/kjoelovelife/ROSKY2/blob/main/install_script/rosky2_dependencies.sh) to install dependenices. Please type command below on the terminal to install:

```bash=
source ~/ROSKY2/install_script/rosky2_dependencies.sh
``` 

## 4. build project ROSKY2 workspace
This step is the last step and make sure you have done:
1. [Write Image to the microSD Card](https://hackmd.io/p3Exut9cTGi54Uw9qBQGpQ#1-Write-Image-to-the-microSD-Card)
2. [Install Ubuntu 20.04 on ROSKY](https://hackmd.io/p3Exut9cTGi54Uw9qBQGpQ#2-Install-Ubuntu-2004-on-ROSKY)
3. [Install ROS1 and ROS2 through ros_menu and dependencies for project ROSKY2](https://hackmd.io/p3Exut9cTGi54Uw9qBQGpQ#3-Install-ROS1-and-ROS2-through-ros_menu-and-dependencies-for-project-ROSKY2)

Ok! Now open the terminal and choose **2** to run the ROS2 environment, and then type command below:

```bash=
cd ~/ROSKY2/ros2_ws && colcon build --symlink-install
```

You may see the error:

![build_workspace_and_error](https://i.imgur.com/022brv1.jpg)

Don't worry! You can type ``` colcon build --symlink-install ``` again and then everythings will be fine.

![build_workspace_again](https://i.imgur.com/htpsz8H.png)


Finally, we will edit cmds in ~/ros_menu/config.yaml to let terminal can find the project ROSKY2. Please type command below that help you to edit:

```bash=
cd ~/ROSKY2/setup/python_scripts && python3 config.py
```

![edit_config](https://i.imgur.com/mHAG5jH.png)

We finish setting up! Now can visit [1_wall_follower.md](https://github.com/kjoelovelife/ROSKY2/blob/main/readme_resource/1_wall_follower.md) to use ROS2 to control ROSKY!