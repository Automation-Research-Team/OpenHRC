# ohrc_teleoperation

The `ohrc_teleoperation` package provides diverse teleoperation interfaces for OpenHRC. These are built on the Interface class in `ohrc_control/interface.h`.

## Rule for launch files

To launch files, use the following command:
```
roslaunch ohrc_teleoperation (interface)_teleoperation.launch robot:=(robot) controller:=(controller)
```

The table below lists the currently tested and implemented options for the parameters:
|argument | options|
|:---|:---|
|interface|marker, omega, state_topic, twist_topic, joy_topic |
|robot|ur5e, fetch, seed, mycobot, toroboarm, crane_x7|
|controller|vel, vel_trj, pos_trj|

Please refer to the examples provided to understand each option in detail.

## Example - Teleoperation using interactive marker


### Velocity level controller (Recommended)
To effectively control a robot, a velocity-level controller is recommended.

#### UR5e
The UR5e default controller is `JointTrajectoryController`. If you need to load `JointVelocityController` in the controller list configuration, refer to the modification file for the gazebo simulation located in the `./example` directory. If you require the original (default) controller settings due to issues like low control frequency (< 100 Hz), please refer to the Trajectory level controller section.

Use the following commands to launch the modified gazebo simulation with UR5e and the teleoperation controller:
```
# launch modified gazebo simulation with UR5e
$ roslaunch ohrc_hw_config ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```

If using real hardware with [`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver), ensure that the loaded controller is checked. You may need to stop the default trajectory controller and start the loaded velocity controller [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/7b6b62bf81f2a032e0b6c7c8e1046cae35e079c7/ur_robot_driver/config/ur5e_controllers.yaml#L129).


### Trajectory level controller
If your robot controller can only accept low-frequency commands (< 100), you may send a trajectory (position) command instead of a velocity command. While the trajectory controller is theoretically more stable than the velocity controller, it can take considerable time to adjust the parameters for smoother robot movement.

If you're patient, you can achieve superior motion, as demonstrated in the following paper:

>Jun Nakanishi, Shunki Itadera, Tadayoshi Aoyama & Yasuhisa Hasegawa (2020) Towards the development of an intuitive teleoperation system for human support robot using a VR device, Advanced Robotics, 34:19, 1239-1253, DOI: 10.1080/01691864.2020.1813623 

#### UR5e
If you use the original ur5e (6 DoF) configuration, please run 
```
# launch original gazebo simulation with UR5e
$ roslaunch ur_gazebo ur5e_bringup.launch

# launch one of the following teleoperation controllers using a topic interface of JointTrajectoryController
# 1) angle position-based trajectory controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch controller:=pos_trj

# Or 2) angle velocity-based trajectory controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch controller:=vel_trj
```
The two controllers differ in the method of solving IK. The first is based on IK finding the optimal joint angle, and the second uses differential IK to find the optimal joint angular velocity.

The first controller may often fail to solve the IK problem within the predefined time period in our current implementation. The second seems more efficient in terms of computational cost. However, in a low-frequency system, the second controller may cause large errors or oscillations due to numerical integration used to obtain the target angle from angular velocity. In such cases, please try using the IK-based controller.


#### Fetch
If you use Fetch robot (8 DoF) (which is not included in this package installation), please run
```
## install gazebo simulation of Fetch Robot like
$ cd ~/catkin_ws/src
$ git clone -b gazebo11 https://github.com/ZebraDevs/fetch_gazebo.git 
$ cd fetch_gazebo
$ rosdep install -i -y --from-paths ./ 
$ catkin build -DCMAKE_BUILD_TYPE=Release

# launch original gazebo simulation with Fetch
$ roslaunch fetch_gazebo simulation.launch

# launch the following teleoperation controller using the action interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation marker_teleoperation.launch robot:=fetch controller:=vel_trj
```
This example includes a position-based trajectory controller (`controller:=pos_trj`) as well.

#### Seed-noid (SEED-R7)
If you use SEED-R7 (only left arm, 7 DoF) (which is not included in this package installation), please run
```
## install gazebo simulation of Fetch Robot like
$ cd ~/catkin_ws/src
$ git clone https://github.com/seed-solutions/seed_smartactuator_sdk
$ git clone https://github.com/seed-solutions/seed_r7_ros_pkg.git
$ cd seed_r7_ros_pkg
$ rosdep install -i -y --from-paths ./ 
$ catkin build -DCMAKE_BUILD_TYPE=Release

# launch gazebo simulation with Seed
$ roslaunch seed_r7_gazebo seed_r7_empty_world.launch

# launch the following teleoperation controller using topic interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation marker_teleoperation.launch robot:=seed controller:=vel_trj
```

#### myCobot 
If you use myCobot (which is not included in this package installation), please run
```
## install gazebo simulation of myCobot Robot
$ cd ~/catkin_ws/src
$ git clone https://github.com/Tiryoh/mycobot_ros.git
$ cd mycobot_ros
$ rosdep install -i -y --from-paths ./ 
$ catkin build -DCMAKE_BUILD_TYPE=Release

# launch gazebo simulation with myCobot
$ roslaunch mycobot_gazebo mycobot_with_emptyworld.launch

# launch the following teleoperation controller using topic interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation marker_teleoperation.launch robot:=mycobot controller:=vel_trj
```

#### crane_x7
If you use crane_x7 (which is not included in this package installation), please run
```
## install gazebo simulation of crane_x7
## official instruction: https://github.com/rt-net/crane_x7_ros
$ cd ~/catkin_ws/src
$ git clone https://github.com/rt-net/crane_x7_ros.git
$ git clone https://github.com/rt-net/crane_x7_description.git
$ rosdep install -i -y --from-paths ./ 
$ catkin build -DCMAKE_BUILD_TYPE=Release

# launch gazebo simulation with crane_x7 (removed MoveIt! part from the original launch file)
$ roslaunch ohrc_hw_config crane_x7_wtih_table.launch

# launch the following teleoperation controller using topic interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation marker_teleoperation.launch robot:=crane_x7 controller:=vel_trj
```

## Example - Teleoperation using geometry_msgs/twist topic
This subscribes twist topic as a commanded velocity of a robot end-effector.
```
$ rosrun (some nodes to generate command velocity)
$ roslaunch ohrc_teleoperation twist_topic_teleoperation.launch robot:=(robot) controller:=(controller)
```


## Example - Teleoperation using sensor_msgs/joy topic
This subscribes joy topic as a velocity command of a robot end-effector.
If you have 3D mouse (spacenav) http://wiki.ros.org/spacenav_node
```
### install spacenav node
$ sudo apt install spacenavd
$ sudo apt install ros-indigo-spacenav-node

$ roslaunch ohrc_teleoperation joy_topic_teleoperation.launch spacenav:=true robot:=(robot) controller:=(controller)
```
The arg `spacenav:=true` launch spacenav node with specific options.
The second button (usually RIGHT button) is used as a trigger for resetting robot's pose.



## Example - Teleoperation using haptic interface `Omega`

```
$ roslaunch ohrc_teleoperation omega_teleoperation.launch robot:=(robot) controller:=(controller)
```





---
## Development
### How to teleoperate a robot from another interface
The easiest way is just to develop a node publishing a topic of the target end-effector state.
The node is assumed that

1. the state topic is published as ``/state`` in a type of ``ohrc_msgs/state``.
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
- The translational motion is relative, and the rotational motion is absolute, which means that the user can do "indexing" for translational motion by repeating enable and disable. 
- The motion is enabled while ``enabled`` in ``ohrc_msgs/state`` is ``true``.
- The reference position of the relative translation is defined when the ``enabled`` if switched from `false` to `ture`.

If you want to add any modifications but still want to use `ohrc_msgs/state` topic, please make another interface referring to `StateTopicInterface` class. (see `omega_interface.cpp`). For example, this way can add a synchronous feedback function.

If you want to develop from a lower scratch, e.g., calling API of the interface(device) synchronously without pub/sub `ohrc_msgs/state` topic, please develop another interface node referring to `MultiCartController` class. (see `marker_interface.cpp`).

---
## Author
Shunki Itadera, Ph.D. (s.itadera@aist.go.jp) - Researcher at ART, ICPS, AIST
