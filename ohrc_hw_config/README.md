# ohrc_hw_config

This package stores hardware configuration files for OpenHRC, which are tested and verified on either the real robot or the Gazebo simulation.

The naming rule for the configuration files is as follows:

```
ohrc_hw_config/(robot)/(robot)_control_config_(publisher).yaml
```

The table below lists the currently implemented options for the parameters:
|argument | options|
|:---|:---|
|robot|ur5e, fetch, seed, mycobot, toroboarm, crane_x7|
|publisher|vel, vel_trj, pos_trj|

Note : Not all argument combinations are valid. For example, the `mycobot` robot only supports the `pos_trj` and `vel_trj` controllers.

In the configuration yaml file, the following parameters can be defined:
|parameter|description|
|:---|:---|
|chain_start|Base frame of the published controller command.|
|chain_end|End-effector frame of the controller command. This will be used as the target frame in IK.|
|root_frame|This will be used as the root frame in IK.|
|controller|Controller (how to solve the joint angle/angular velocity in IK) ("Position" or "Velocity") |
|publisher|Type of command topic type subscribed by controller ("Position" or "Velocity" or "Trajectory" or "TrajectoryAction") |
|topic_namespace|Namespace of the command topic.|
|solver|Used IK solver implementation|
|self_collision_avoidance|Whether to enable self-collision avoidance.(bool)|
|control_freq|Frequency of the controller.|
|initIKAngle|Initial joint angles for IK solver. Its size should be equal to the robot's DoF|
|initial_pose_frame|Frame of the initial pose of the end-effector.(same as chain_start)|
|initial_pose|Initial pose of the end-effector.|
|follower_list|List of the robot namespaces.|
