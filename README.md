![GitHub release (latest by date)](https://img.shields.io/github/v/release/itadera/OpenHRC)
[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)
![GitHub](https://img.shields.io/github/license/itadera/OpenHRC)
# OpenHRC
Open Human-Robot Collaboration/Cooperation Library

Aiming to enhance research on human-robot interaction, we develop this open ROS package for offering easy implementation of HRC software such as interaction and teleoperation.
OpenHRC includes some tools for HRC like a robot controller for multiple robots with (self-)collision avoidance, human skeleton detection, imitation learning and so on. We hope this package helps you implement your HRC ideas instantly.

## Requirements
OpenHRC has been developed and tested in the following environments:
- Development Environment: Ubuntu 20.04 (ROS Noetic)
- Tested Environments:
  - Ubuntu 20.04 (native, Docker)
  - Ubuntu 22.04 (Docker)
  - Windows 11 (Docker on WSL2)
  - macOS (Docker)

Currently, this library is not compatible with ROS2. Although there are intentions to port this package to ROS2, there is no specific plan in place.


## Native (Ubuntu 20.04) / Docker (on Ubuntu distribution) Installation

If you just try OpneHRC with minimum effor on other operating systems, please take a look at [Docker-VNC Installation](#Docker-VNC-Installation) section.

In the following instruction, the catkin workspace directory is assumed to be `~/catkin_ws` on host.

### Clone the Source Code
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/itadera/OpenHRC.git 
```

### (Option) Install and Run Docker with Rocker

```
$ cd ~/catkin_ws/src/OpenHRC
$ sudo sh install-docker-rocker.sh
$ rocker --nvidia --x11 --user --home osrf/ros:noetic-desktop-full
```

This instruction uses a docker image offered by OSRF, including minimum ROS setting, and comples OpenHRC manuallly. 
If your PC has no NVIDIA GPU, please remove `--nvidia` option.

### Resolve Dependencies
```
$ cd ~/catkin_ws/src/OpenHRC

### Clone submodule sources
$ git submodule update --init --recursive

### Install dependency packages
$ rosdep update
$ rosdep install -i -y --from-paths ./ 
```

### Build
Compile with `catkin-tools`, which should be installed as a dependency above:

```
$ cd ~/catkin_ws
$ catkin build -DCMAKE_BUILD_TYPE=Release
```
Note: The default tool `catkin_make` cannot compile non-ROS code.


## Docker-VNC Installation
If you want to install OpenHRC natively on Ubuntu 20.04, please see the [Native Installation](#Native-Installation-(Ubuntu-20.04)) section and skip this section.

This package can be tested in a Docker container, which should work on Linux, Windows, and macOS. The Dockerfile is based on https://github.com/Tiryoh/docker-ros-desktop-vnc.

### Install Docker
If you are using Linux (Ubuntu) or WSL2 on Windows, please run:
```
$ sudo apt install -y curl
$ curl -s https://raw.githubusercontent.com/itadera/OpenHRC/main/install-docker-rocker.sh | /bin/bash
$ sudo service docker start
```

If you encounter an issue with `permission denied`, please try:
```
sudo chmod 666 /var/run/docker.sock
```

This install instruction uses Docker Engine, not Docker Desktop, which is not free for commercial use. Both options are compatible, but Docker Engine may offer better performance. If you prefer Docker Desktop, please follow the instructions at https://docs.docker.com/desktop/.


On other OS such as native Windows and macOS, please install Docker Desktop from https://www.docker.com/products/docker-desktop and start it.

The following commands are excuted inside of Docker Container.

### Clone Sources
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/itadera/OpenHRC.git 
```

### Build Docker Image
```
$ cd OpenHRC
$ docker build -t openhrc-vnc:noetic . --no-cache
```

### Run Docker 
```
$ docker run --rm -it -p 10000:10000 -p 5005:5005 -p 6080:80 --shm-size=512m openhrc-vnc:noetic
```
You can now access the desktop GUI at 
http://localhost:6080/


## Getting Started
To test either the native or Docker installation, you can first try the teleoperation node with interactive markers for UR5e.

Open a terminal and run:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch ohrc_hw_config ur5e_bringup.launch
```
This command launches the UR5e simulation on Gazebo.

Open another terminal (if on Docker, run `docker exec <container ID> /bin/bash`) and run:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```

This command starts the robot controller, allowing you to operate the end-effector using an interactive marker on Rviz.


## Tutorials

1. Teleoperation library: [ohrc_teleoperation](./ohrc_teleoperation)
2. Imitation Learning library: [ohrc_imitation_learning](./ohrc_imitation_learning)



## Citation

If you use this package in your academic research, we would appreciate it if you could cite the following paper.
>(comming soon)


## License
This software is released under dual license of LGPL-v2.1 and individual license.
About the individual license, please make a contact to the author.

## Author
Shunki Itadera (https://staff.aist.go.jp/s.itadera/) - Researcher at ART, ICPS, AIST

We welcome any feedback and contributions. Please feel free to contact us if you have any questions or comments.

Besides, we are looking for research collaborators and students who are interested in Human-Robot Interaction using OpenHRC. If you are interested, please send me an email.