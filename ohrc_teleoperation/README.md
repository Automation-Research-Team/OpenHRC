# ohrc_teleoperation

## Example - Teleoperation using interactive marker

### Velocity level controller (Recommended)
The best way would be to control a robot at a velocity level.

#### UR5e
Since the default controller of UR5e is `JointTrajectoryController`, we need to load `JointVelocityController` in the controller list configuration.
The modification file for the gazebo simulation can be found in `./example` directory.
If you want to use the original (default) controller settings due to like low control frequency issue (< 100 Hz), please see next (Trajectory level controller). 


```
# launch modified gazebo simulation with UR5e
$ roslaunch ohrc_teleoperation ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation ur5e_marker_teleoperation.launch
```

If you use this with the real hardware with [`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver), you need to check the loaded controller. (Probably, you just need to stop the default trajectory controller and start the loaded velocity controller [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/7b6b62bf81f2a032e0b6c7c8e1046cae35e079c7/ur_robot_driver/config/ur5e_controllers.yaml#L129))


### Trajectory level controller
If your robot controller can only accept low-frequency command (< 100), you can send a trajectory (position) command instead of a velocity command.
Theoretically, this trajectory controller is stabler than the velocity controller,
However, in practice, you would need to spend much time adjusting many parameters to make the robot movement smoother.

If you are patient, you can achieve better motion like the following paper;

>Jun Nakanishi, Shunki Itadera, Tadayoshi Aoyama & Yasuhisa Hasegawa (2020) Towards the development of an intuitive teleoperation system for human support robot using a VR device, Advanced Robotics, 34:19, 1239-1253, DOI: 10.1080/01691864.2020.1813623 


#### UR5e
If you use the original ur5e (6 DoF) configuration, please run 
```
# launch original gazebo simulation with UR5e
$ roslaunch ur_gazebo ur5e_bringup.launch

# launch one of the following teleoperation controllers using a topic interface of JointTrajectoryController
# 1) angle position-based trajectory controller
$ roslaunch ohrc_teleoperation ur5e_marker_teleoperation_pos_trj.launch

# Or 2) angle velocity-based trajectory controller
$ roslaunch ohrc_teleoperation ur5e_marker_teleoperation_vel_trj.launch
```
The difference between the two controllers is the way of solving IK. The first is based on IK finding the optimal joint angle, and the second is based on differential IK finding the optimal joint angular velocity.

In the current implementation, the first controller often fails to solve the IK problem within the predefined time period. The second one seems much useful in term of calculation cost.
However, in a case of a low-frequency system, the second controller brings a large error or oscillation because there is a numerical integration to get the target angle from the angler velocity. In this case, please try the IK-based controller.


#### Fetch
If you use Fetch robot (8 DoF) (which is not included in this package installation), please run
```
## install gazebo simulation of Fetch Robot like
$ cd ~/catkin_ws/src
$ git clone -b gazebo11 https://github.com/ZebraDevs/fetch_gazebo.git 
$ cd fetch_gazebo
$ rosdep install -i -y --from-paths ./ 
$ catkin_make -DCMAKE_BUILD_TYPE=Release

# launch original gazebo simulation with Fetch
$ roslaunch fetch_gazebo simulation.launch

# launch the following teleoperation controller using the action interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation fetch_marker_teleoperation_vel_trj.launch
```
This example includes a position-based trajectory controller (`fetch_marker_teleoperation_pos_trj.launch`) as well.

#### Seed-noid (SEED-R7)
If you use SEED-R7 (only left arm, 7 DoF) (which is not included in this package installation), please run
```
## install gazebo simulation of Fetch Robot like
$ cd ~/catkin_ws/src
$ git clone https://github.com/seed-solutions/seed_smartactuator_sdk
$ git clone https://github.com/seed-solutions/seed_r7_ros_pkg.git
$ cd seed_r7_ros_pkg
$ rosdep install -i -y --from-paths ./ 
$ catkin_make -DCMAKE_BUILD_TYPE=Release

# launch gazebo simulation with Seed
$ roslaunch seed_r7_gazebo seed_r7_empty_world.launch

# launch the following teleoperation controller using topic interface of JointTrajectoryController
$ roslaunch ohrc_teleoperation fetch_marker_teleoperation_vel_trj.launch
```


---
## Development
### How to teleoperate a robot from another interface
The easest way is just to develop a node publishing a topic of the target end-effector state.
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
- The translational motion is relative, and the rotational motion is absolute, which means that the user can do "indexing" for translational motion by repeating enable and disable. 
- The motion is enabled while ``enabled`` in ``ohrc_msgs/state`` is ``true``.
- The reference position of the relative translation is defined when the ``enabled`` if switched from `false` to `ture`.

If you want to add any modifications but still want to use `ohrc_msgs/state` topic, please make another interface referring to `StateTopicInterface` class. (see `omega_interface.cpp`). For example, this way can add a synchronous feedback function.

If you want to develop from a lower scratch, e.g., calling API of the interface(device) synchronously without pub/sub `ohrc_msgs/state` topic, please develop another interface node referring to `MultiCartController` class. (see `marker_interface.cpp`).

---
## Author
Shunki Itadera, Ph.D. (s.itadera@aist.go.jp) - Researcher at ART, ICPS, AIST
