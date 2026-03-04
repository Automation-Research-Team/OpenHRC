Here we list the best practices for using real robot hardware with OpenHRC.

## UR5e (Universal Robots)

1. Install `Universal Robots ROS2 Driver` to your PC and setup your robot

see https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver


2. Start robot controller
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=xxx.xxx.xxx.xxx initial_joint_controller:=forward_velocity_controller
```

3. Start ohrc_teleoperation
```bash
ros2 launch ohrc_teleoperation marker_teleoperation.launch.py robot:=ur5e
```


## Franka Research 3 (Franka Robotics)

1. Install libfranka and franka_ros2

We followed this installation guide https://github.com/itadera/libfranka_ros2

2. Build joint-velocity-based impedance controller
   
see https://github.com/OpenHRC/ohrc_fr3_controllers

3. update the robot configuration file

4. Start robot controller
```bash
ros2 launch ohrc_fr3_controllers bringup.launch.py 
```

5. Start ohrc_teleoperation
```bash
ros2 launch orhc_teleoperation marker_teleoepration.launch.py robot:=fr3
```


## xArm 6 (UFactory)
1. Install ROS2 driver

see https://github.com/xArm-Developer/xarm_ros2


2. Start robot controller
   
3. Start ohrc_teleoperation
```bash
ros2 launch orhc_teleoperation marker_teleoepration.launch.py robot:=fr3
```


## Gen 3 Lite (Kinova Robotics)
1. install ROS2 driver

2. Start robot controller
   
3. Start ohrc_teleoperation
```bash
ros2 launch orhc_teleoperation marker_teleoepration.launch.py robot:=gen3lite
```

## LBR iiwa 14 R820 (KUKA)

1. install ROS2 driver

2. Start robot controller
   
3. Start ohrc_teleoperation
```bash
ros2 launch orhc_teleoperation marker_teleoepration.launch.py robot:=iiwa14
```

## SO-ARM 101 (Hugging Face)
1. install ROS2 driver

2. Start robot controller
   
3. Start ohrc_teleoperation
```bash
ros2 launch orhc_teleoperation marker_teleoepration.launch.py robot:=so101
```

## Others
We welcome your contributions to add additional robot setup instructions for OpenHRC!