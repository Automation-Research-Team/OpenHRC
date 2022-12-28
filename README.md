# OpenHRC
~~Open~~ Human-Robot Collaboration/Cooperation Library

---
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

# if you build this pkg with cakitn_build_tools
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
