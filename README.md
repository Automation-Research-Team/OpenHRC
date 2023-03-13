# OpenHRC
Open Human-Robot Collaboration/Cooperation Library

[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)
---

## Requirements
This library is tested on the following envirionment:
- Ubuntu 20.04 (ROS Noetic)

This library is not compatible with ROS2 at this moment.
I intent to port this package to ROS2, but have no plan.

## Install

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

### solve dependencies
## install xcb libraries for OpenXR, which are not listed in rosdep
$ sudo apt install libx11-xcb-dev libxcb-dri2-0-dev libxcb-glx0-dev libxcb-icccm4-dev libxcb-keysyms1-dev libxcb-randr0-dev

## install other libraries using rosdep
$ rosdep update
$ rosdep install --ignore-src -y -r --from-paths ./
```

### build
Compile with catkin-tools, which should be installed above.

```
$ catkin build -DCMAKE_BUILD_TYPE=Release
```
Note that, the default tool `catkin_make` cannot compile non-ros codes.


---
## Tutorials

1. Teleoperation library: [ohrc_teleoperation](./ohrc_teleoperation)
1. Imitation Learning library [ohrc_imitation_learning](./ohrc_imitation_learning)



---
## Author
Shunki Itadera, Ph.D (s.itadera@aist.go.jp) - Researcher at ART, ICPS, AIST