# Setup operating environment for ROSKY2 with recomputer-J1010
This note will help you how to setup the Project "ROSKY2" with reComputer-J1010.

## Hardware you need to prepare
There are some things you have to prepare:
- [reComputer-J110](https://www.icshop.com.tw/product-page.php?28703)
- Mouse, Keyboard and Monitor with HDMI port
- USB flash drive which memory space is **>= 64 GB**



## 1. Check the system 
### 1-1. Initialize your system
You can quickly install system when the first time you get reComputer-J1010. If you don't know how to initialize, please check this site [Initial system start-up](https://wiki.seeedstudio.com/reComputer_getting_started/#initial-system-start-up), or the Mandarin version here [reComputer Jetson 系列-入門與系統操作教學](https://www.circuspi.com/index.php/2022/07/05/recomputer-jetson-operating-system/)

### 1-2. Chceck the memory space
There is a eMMC inside the reComputer-J1010, but the space is 16 GB too small to isntall additional software.
The first thing need to to do is expanding the memory space. Please prepare a USB whcih is **>= 64GB** and then follow the steps on this site to do the [reComputer for Jetson Memory Expansion](https://wiki.seeedstudio.com/reComputer_Jetson_Memory_Expansion/), or you can read the Mandarin version here: [reComputer Jetson 系列擴充儲存空間](https://www.circuspi.com/index.php/2022/07/07/recomputer-jetson-storage/)

You can use the command below on terminal to check the memory space:
```bash=
$ df -h
```

The image below is my memory space in the reComputer-J1010, and the Size of /dev/sda1 is 59G:

![memory space in reComputer-J1010](https://i.imgur.com/zzoeXui.png)


### 1-3. Check the Jetpack version
Open the terminal and check the Jetpack version, please type the command below:
```bash=
$ sudo apt-cache show nvidia-jetpack
```
The Jetpack version must be **>= 4.6**, and the image below is my Jetpack version


![Jetpack version](https://i.imgur.com/iQ9qCSy.png)

## 2. Install ROS-melodic and ROS2-Foxy in reComputer-J1010
Accoring to the [NVIDIA's roadmap](https://developer.nvidia.com/embedded/develop/roadmap) and the post on  [NVIDIA's forums](https://forums.developer.nvidia.com/t/jetson-software-roadmap-for-2h-2021-and-2022/177724), you just can use Ubuntu 18.04 on Jetson nano right now through the offical image. That means you just can isntall [ROS2-Foxy from source code](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) in the offical image.


Please feel free to check the project [ros2_development_tools](https://github.com/kjoelovelife/ros2_development_tools), you can find how to use script inside to install ROS and ROS2 from source code. 


## 3. Download the project ROSKY2
Awesome! After installing ROS2-Foxy, you have to donwload the projcet ROSKY2 through git. Please open the terminal and type the command below to clone the repository of ROSKY2:
```bash=
$ git clone https://github.com/kjoelovelife/ROSKY2
```

Now you can find the project ROSKY2 in your $HOME directory.


## 4. Setup the operating environment for the project ROSKY2 
Once you download the project ROSKY2, next step is setting up the operating environment or you can't run the project ROSKY2.

There are many dependencies with the project ROSKY2, it waste time to manually install those. You can use a script to isntall those automatically. Please open the terminal and type the command below to run the script:
```bash=
$ ./ROSKY2/install_script/rosky2_dependencies.sh
```
All right. Now everything is ready to run the ROSKY2. Before doing that, I want to introduce a convenient tool, [ros_menu](https://github.com/Adlink-ROS/ros_menu). It's from ADLINK and can help you to choose the operating enviroment you want to use. Now you can use ros_menu after successfully running the script **"rosky2_dependencies.sh"**.

Please re-open the terminal and then you can find your termial will be the similar image below:

![ros_menu](https://i.imgur.com/Wwql4dL.png)

You can type the number to choose the environment you want, you will type **2** to choose the environment while you want to run the project ROSKY2.
The image below is an example for ready to run the project ROSKY2 with reComputer-J1010.

![ready to run the project ROSKY2](https://i.imgur.com/5ET2MjU.png)

## Next

User can continue the note -> [CH 1: ROSKY2 - remotely log in](1_remotely_log_in.md)

Please have fun to use ROSKY2 :)