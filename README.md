# ROSKY2 information

This project is about how to use ROS2 to control [ROSKY2](https://www.icshop.com.tw/product-page.php?28182)

![ROSKY2](https://i.imgur.com/WU13drn.jpg)


## Information

- Hardware
  - Single Computer Board(SBD): [Jetson Nano 4G Developer Kit](https://www.icshop.com.tw/product-page.php?27812)
  - Platform: mecanum
  - Lidar: [Yd-lidar X4](https://www.icshop.com.tw/product-page.php?26030)
  - Micro-SD card: [Micro SD 64GB](https://www.icshop.com.tw/product-page.php?27389)
  - Wi-fi module: [Intel8265AC](https://www.icshop.com.tw/product-page.php?27325)

- Software
  - Operating System: [Xubuntu 20.04 Focal Fossa L4T R32.3.1](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)
  - Framework:
    - Robot Operating System 2(ROS2) - Foxy
  - Python: 3.8.10

## Developer

- Wei-Chih Lin(weichih.lin@protonmail.com)

## How to use
Suggest visiting in order:

| Example Name | Description  |
| ------------ | -------------|
| [0. setup environment](https://github.com/CIRCUSPi/ROSKY2/blob/main/readme_resource/0_setup_environment.md) | use simple command to setup operating environment.     |
| [1. wall_follower](https://github.com/kjoelovelife/ROSKY2/blob/main/readme_resource/1_wall_follower.md)| Learn how to use ROS2 to control ROSKY |
| 2. rosky2_slam | to be continued ...  |
| 3. rosky2_nav  | to be continued ...  |

## Update Log

### 2021/10/18
> Release version 1.0.0, can use ROSKY2 to learn the concept of ROS2.
> More information can visit 0_setup_environment.md and 1_wall_follower.md in readme_resourse. 

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