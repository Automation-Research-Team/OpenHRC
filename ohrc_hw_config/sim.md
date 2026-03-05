Here we list the best practices for controlling simulated robots with OpenHRC.


## UR series (Universal Robots)

#### 1. Install the robot simulation package (ur_simulation_gz)

UR simulation is automatically installed in OpenHRC build instruction. If you somehow failed to install it, please run
```bash
sudo apt install ros2-humble-ur-simulation-gz
```

#### 2. Start simulation
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:={UR_TYPE} initial_joint_controller:=forward_velocity_controller launch_rviz:=false
```
where the target robot type `UR_TYPE` needs to be chosen from `{ur3, ur5, ur10, ur3e, ur5e, ur7e, ur10e, ur12e, ur16e, ur8long, ur15, ur18, ur20, ur30}`.

e.g., if you simulate ur5e, please run
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e initial_joint_controller:=forward_velocity_controller launch_rviz:=false
```

#### 3. Start ohrc_teleoperation
```bash
ros2 launch ohrc_teleoperation marker_teleoperation.launch.py robot:={UR_TYPE}
```
where the robot type needs to be same as the launched as simulation model.

e.g., if you teleoperation ur5e, please run
```bash
ros2 launch ohrc_teleoperation marker_teleoperation.launch.py robot:=ur5e
```



## Franka Research 3 (Franka Robotics)


#### 1. Install simulation package

#### 2. Start simulation


#### 3. Start ohrc_teleoperation


## xARM series (UFactory)

#### 1. Install simulation package

#### 2. Start simulation


#### 3. Start ohrc_teleoperation



## Gen 3 series (Kinova Robotics)
#### 1. Install simulation package
#### 2. Start simulation
#### 3. Start ohrc_teleoperation

## LBR iiwa and med series (KUKA)
#### 1. Install simulation package
#### 2. Start simulation
#### 3. Start ohrc_teleoperation

## Tiago Pro (PAL Robotics)
#### 1. Install simulation package
#### 2. Start simulation
#### 3. Start ohrc_teleoperation

## CRANE-X7 (RT Cooperation)
#### 1. Install simulation package
#### 2. Start simulation
#### 3. Start ohrc_teleoperation

## Torobo (Tokyo Robotics)
#### 1. Install simulation package
#### 2. Start simulation
#### 3. Start ohrc_teleoperation    
        



## Others
We welcome your contributions to add additional robot setup instructions for OpenHRC!