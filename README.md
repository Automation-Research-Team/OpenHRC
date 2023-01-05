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

### clone submodule source
$ git submodule update --init --recursive

### install dependency packages
$ rosdep update
$ rosdep install -i -y --from-paths ./ 
```

### build
```
### if you build this pkg with catkin_make
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release

### if you build this pkg with catkin_build_tools
$ catkin build -DCMAKE_BUILD_TYPE=Release
```


---
## Example

### Teoperation with UR5e using interactive marker

#### Velocity level contorller (Recommended)
Note that, the defalt controleller of UR5e is `JointTrajectoryController`, we need to load `JointVelocityController` in controller list configulation.
The modification file for gazebo simulation can be found in `./example` directory.
If you want to use the original (default) controller settings, please see next (Trajectory level controller). 


```
# launch modified gazebo simulation with UR5e
$ roslaunch ohrc_teleoperation ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```

If you use this with the real hardware with [`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver), you need to check loaded controller. (Probably, you just need to stop the default trajectory controller and start velocity controller which is already loaded [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/7b6b62bf81f2a032e0b6c7c8e1046cae35e079c7/ur_robot_driver/config/ur5e_controllers.yaml#L129))


#### Trajectory level controller



```
# launch original gazebo simulation with UR5e
$ roslaunch ur_gazebo ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```


---
## Development
### How to teleoperate a robot from another interface
Easest way is just to develop a node publishing a topic of the target end-effector state.
The node is assumed that

1. the state topic is published as ``/state`` in type of ``ohrc_msgs/state``.
1. the state is defined in ``world``.

Then, launch the following nodes: 
```
# launch gazebo simulation (e.g., with UR5e)
$ roslaunch ohrc_teleoperation ur5e_bringup.launch

# run the made node publishing 
$ rosrun <develoeped node> 

# launch teleoperation controller
$ roslaunch ohrc_teleoperation state_topic_teleoperation.launch
```

Note that,
- The translational motion is relative, and the rotational motion is absolute.
- The motion is enabled while ``enabled`` in ``ohrc_msgs/state`` is ``true``.
- The reference position of the relative translation is defined when the ``enabled`` if swtiched from `false` to `ture`.

If you want to add any modification, but still want to use `ohrc_msgs/state` topic, please make another interface refering to `StateTopicInterface` class. (see `omega_interface.cpp`). For example, this way can add synchronous feedback function.

If you want to develop from a lower scratch, e.g., calling API of the interface(device) synchronously without pub/sub `ohrc_msgs/state` topic, please develop another interface node refering to `MultiCartController` class. (see `marker_interface.cpp`).


