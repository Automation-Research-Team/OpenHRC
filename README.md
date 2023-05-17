# OpenHRC
Open Human-Robot Collaboration/Cooperation Library

[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)

---

## Requirements
OpenHRC has been developed and tested in the following environments:
- Development Environment: Ubuntu 20.04 (ROS Noetic)
- Tested Environments:
  - Ubuntu 20.04 (native, Docker)
  - Ubuntu 22.04 (Docker)
  - Windows 11 (Docker on WSL2)
  - macOS (Docker)

Currently, this library is not compatible with ROS2. Although there are intentions to port this package to ROS2, there is no specific plan in place.

---
## Native Installation (Ubuntu 20.04)

For other operating systems, please see the [Docker installation](#docker-install) section.

The catkin workspace directory is assumed to be `~/catkin_ws`.

### Clone the Source
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/itadera/OpenHRC.git 
```

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
Compile with catkin-tools, which should be installed as a dependency:

```
$ catkin build -DCMAKE_BUILD_TYPE=Release
```
Note: The default tool `catkin_make` cannot compile non-ROS code.

---
## Docker Installation
This package can be tested in a Docker container, which should work on Linux, Windows, and macOS. The Dockerfile is based on https://github.com/Tiryoh/docker-ros-desktop-vnc.

### Install Docker
If you are using Linux (Ubuntu) or WSL2 on Windows, please run:
```
$ sudo apt install -y curl
$ curl -s https://raw.githubusercontent.com/karaage0703/ubuntu-setup/master/install-docker.sh | /bin/bash
$ sudo service docker start
```

If you encounter a permission denied issue, please try:
```
sudo chmod 666 /var/run/docker.sock
```

This method uses Docker Engine, not Docker Desktop, which is not free for commercial use. Both options are acceptable, but Docker Engine may offer better performance. If you prefer Docker Desktop, please follow the instructions at https://docs.docker.com/desktop/.

### Clone Sources
```
$ git clone https://github.com/itadera/OpenHRC.git 
$ cd OpenHRC
$ git submodule update --init --recursive
```

### Build Docker Image
```
$ docker build -t openhrc:noetic . --no-cache
```

### Run Docker 
```
$ docker run --rm -it -p 10000:10000 -p 5005:5005 -p 6080:80 --shm-size=512m openhrc:noetic
```
You can now access the desktop GUI at: 
http://localhost:6080/

---
## Getting Started
To test either the native or Docker installation, you can try the teleoperation node with interactive markers for UR5e.

Open a terminal and run:
```
$ roslaunch ohrc_teleoperation ur5e_bringup.launch
```
This command launches the UR5e simulation on Gazebo.

Open another terminal and run:
```
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```
This command starts the robot control, allowing you to operate the end-effector using an interactive marker.

---
## Tutorials

1. Teleoperation library: [ohrc_teleoperation](./ohrc_teleoperation)
2. Imitation Learning library: [ohrc_imitation_learning](./ohrc_imitation_learning)


---
## Citation

If you use this package in your academical research, we would appreciate it if you could cite the following paper.


>(comming soon)


---
## Author
Shunki Itadera (https://staff.aist.go.jp/s.itadera/)