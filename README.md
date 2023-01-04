# OpenHRC
~~Open~~ Human-Robot Collaboration/Cooperation Library

[![ROS build workflow](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/itadera/OpenHRC/actions/workflows/build.yaml)
---

## Requirements
This library is tested on the following envirionment:
- Ubuntu 20.04 (ROS Noetic)

And this library is not compatible with ROS2 at this moment.

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

# clone submodule source
$ git submodule update --init --recursive

# install dependency packages
$ rosdep update
$ rosdep install -i -y --from-paths ./ 
```

### build
```
# if you build this pkg with catkin_make
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release

# if you build this pkg with catkin_build_tools
$ catkin build -DCMAKE_BUILD_TYPE=Release
```


---
## Example

### Teoperation with UR5e using interactive marker
```
# launch gazebo simulation with UR5e
$ roslaunch ohrc_teleoperation ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```
