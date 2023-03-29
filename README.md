# OpenHRC
Open Human-Robot Collaboration/Cooperation Library

[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)
---

## Requirements
This library is tested on the following envirionment:
- Ubuntu 20.04 (ROS Noetic)

This library is not compatible with ROS2 at this moment.
I intent to port this package to ROS2, but have no plan.

## Native Install

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
If you use Linux (ubuntu), just run
```
$ sudo apt install -y curl
$ curl -s https://raw.githubusercontent.com/karaage0703/ubuntu-setup/master/install-docker.sh | /bin/bash
```

### clone sources
```
$ git clone https://github.com/itadera/OpenHRC.git 
$ cd OpenHRC
$ git submodule update --init --recursive
```

### build docker container
```
$ docker build -t openhrc:noetic . --no-cache
```

### start docker container
```
$ docker run --rm -it -p 10000:10000 -p 5005:5005 -p 6080:80 --shm-size=512m openhrc:noetic

```
Then, you can access GUI via 
http://127.0.0.1:6080/


---
## Tutorials

1. Teleoperation library: [ohrc_teleoperation](./ohrc_teleoperation)
1. Imitation Learning library [ohrc_imitation_learning](./ohrc_imitation_learning)



---
## Author
Shunki Itadera, Ph.D (s.itadera@aist.go.jp) - Researcher at ART, ICPS, AIST