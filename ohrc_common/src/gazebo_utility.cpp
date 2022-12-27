#include "ohrc_common/gazebo_utility.h"

bool gazebo_utility::checkGazeboInit() {
  // wait for simulation time update
  // see http://gazebosim.org/tutorials?tut=drcsim_ros_cmds&cat=drcsim

  ROS_INFO("Waiting for gazebo launch ...");

  ros::Time last_ros_time_;
  bool wait_gazebosim = true;
  while (wait_gazebosim) {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait_gazebosim = false;

    if (!ros::ok())
      return false;
  }
  ROS_INFO("Gazebo sim ok");

  return true;
}
