# OpenHRC
Open Human-Robot Collaboration/Cooperation Library

[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)
---

## Requirements
OpenHRC is developed on the following envirionment:
- Ubuntu 20.04 (ROS Noetic)

Besides, OpenHRC is tested on the following envirionment:
 - Ubuntu 20.04 (native, docker)
 - Ubuntu 22.04 (docker)
 - Windows 11 (docker on WLS2)
 - Mac OS (docker)

This library is not compatible with ROS2 at this moment.
I intent to port this package to ROS2, but have no plan.

## Native Install (Ubuntu 20.04)

If you use other OS, please see [docker install](#docker-install).

The catkin workspace directry is assumed to be ``~/catkin_ws``.
### clone sourse
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/itadera/OpenHRC.git 
```

### solve dependency
```
$ cd ~/catkin_ws/src/OpenHRC

### clone submodule source
$ git submodule update --init --recursive

### install dependency packages
$ rosdep update
$ rosdep install -i -y --from-paths ./ 
```

### build
Compile with catkin-tools, which should be installed above.

```
$ catkin build -DCMAKE_BUILD_TYPE=Release
```
Note that, the default tool `catkin_make` cannot compile non-ros codes.



## Docker Install
This package can be tested in a docker container which should work on Linux/Windows/Mac.
This dockerfile is based on https://github.com/Tiryoh/docker-ros-desktop-vnc

### install docker
If you use Linux (ubuntu) or WLS2 on Windows, please run
```
$ sudo apt install -y curl
$ curl -s https://raw.githubusercontent.com/karaage0703/ubuntu-setup/master/install-docker.sh | /bin/bash
$ sudo service docker start
```

If you have an issue on permission denied, please try
```
sudo chmod 666 /var/run/docker.sock
```

This way is to use Docker Engine, not Docker Desktop which is not free for commercial use.
Both of them are OK, but Docker Engine can perform better.
If you use Docker Desktop, please follow the instruction on https://docs.docker.com/desktop/.



### clone sources
```
$ git clone https://github.com/itadera/OpenHRC.git 
$ cd OpenHRC
$ git submodule update --init --recursive
```

### build docker image
```
$ docker build -t openhrc:noetic . --no-cache
```

### run docker 
```
$ docker run --rm -it -p 10000:10000 -p 5005:5005 -p 6080:80 --shm-size=512m openhrc:noetic
```
Then, you can access desk top GUI at 
http://localhost:6080/

## Get Start
As a test of either native or docker install, you can try teleoperation node with interactive marker for UR5e.

Open a terminal and run
```
$ roslaunch ohrc_teleoperation ur5e_bringup.launch
```
This launch UR5e simulation on Gazebo.

Open another terminal and run 
```
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```
This start controlling the robot and you can operate the end-effector via an interactive marker.


---
## Tutorials

1. Teleoperation library: [ohrc_teleoperation](./ohrc_teleoperation)
1. Imitation Learning library [ohrc_imitation_learning](./ohrc_imitation_learning)



---
## Author
Shunki Itadera, Ph.D (s.itadera@aist.go.jp) - Researcher at ART, ICPS, AIST
