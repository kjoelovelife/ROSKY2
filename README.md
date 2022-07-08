# ROSKY2 information

This project is about how to use ROS2 to control [ROSKY2](https://www.icshop.com.tw/product-page.php?28182)

![ROSKY2](https://i.imgur.com/WU13drn.jpg)


## Information

- Hardware
  - Single Computer Board(SBD): 
    - [Jetson Nano 4G Developer Kit](https://www.icshop.com.tw/product-page.php?27812)
    - [reComputer-J1010](https://www.icshop.com.tw/product-page.php?28703)
  - Platform: mecanum
  - Lidar: [Yd-lidar X4](https://www.icshop.com.tw/product-page.php?26030)
  - Micro-SD card: [Micro SD 64GB](https://www.icshop.com.tw/product-page.php?27389)
  - Wi-fi module: [Intel8265AC](https://www.icshop.com.tw/product-page.php?27325)

- Software
  - Operating System: 
    - [Xubuntu 20.04 Focal Fossa L4T R32.3.1](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768) with Jetson Nano 4G Developer Kit.
    - [Ubuntu 18.04 with Jetpack 4.6](https://developer.nvidia.com/embedded/jetpack-sdk-46) with reComputer-J1010
  - Framework:
    - Robot Operating System 2(ROS2) - Foxy
  - Python: 
    - 3.8.10 with Jetson Nano 4G Developer Kit.
    - 3.6.9 with reComputer-J1010
  - Source code: [kjoelovelife/ROSKY2](https://github.com/kjoelovelife/ROSKY2) 

## Developer

- Wei-Chih Lin(weichih.lin@protonmail.com)

## How to use
Suggest visiting in order:

| Example Name | Description  |
| ------------ | -------------|
| [0.0 setup environment with Jetson nano 4G Developer Kit](https://github.com/kjoelovelife/ROSKY2/blob/main/readme_resource/0_0_setup_environment_with_jetson_nano.md) | if you use Jetson nano 4G Developer Kit, please use this to setup the operating environment. |
| [0.1 setup environment with reComputer](https://github.com/kjoelovelife/ROSKY2/blob/main/readme_resource/0_1_setup_environment_with_recomputer.md)  |  if you use reComputer-J1010, please use this to setup the operating environment. |
| [1. Teleop ROSKY2](https://hackmd.io/@weichih-lin/ROSKY2_teleop_with_keyboard)           |  teleop ROSKY2 with your keyboard  |
| [2. wall_follower](https://github.com/kjoelovelife/ROSKY2/blob/main/readme_resource/2_wall_follower.md)| Learn how to use ROS2 to control ROSKY |
| 3. rosky2_slam | to be continued ...  |
| 4. rosky2_nav  | to be continued ...  |

## Update Log
### 2022/07/08
> Release version 1.1.0, now can use the platform reComputer-J1010 to run this project.

### 2021/10/18
> Release version 1.0.0, can use ROSKY2 to learn the concept of ROS2.
> More information can check the folder readme_resource

## License and Disclaimer
Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.