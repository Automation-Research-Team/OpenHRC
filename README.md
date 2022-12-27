# OpenHRC
~~Open~~ Human-Robot Collaboration/Cooperation Library

---
## Install
### clone sourse
```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/itadera/OpenHRC.git 
```

### solve dependency
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

### build
```
# if you build this pkg with catkin_make
$ catkin_make -DCMAKE_BUILD_TYPE=Release

# if you build this pkg with cakitn_build_tools
$ catkin build -DCMAKE_BUILD_TYPE=Release
```


---
## Example

### with UR5e
```
# launch gazebo simulation with UR5e
$ roslaunch ohrc_teleoperation ur5e_bringup.launch

# launch teleoperation controller
$ roslaunch ohrc_teleoperation marker_teleoperation.launch
```
