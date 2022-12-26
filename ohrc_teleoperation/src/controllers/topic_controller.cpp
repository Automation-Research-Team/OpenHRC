#include "ohrc_control/topic_controller.hpp"

TopicController::TopicController() {
  subCmd = nh.subscribe<geometry_msgs::Pose>("/" + robot_ns + "cmd_pose", 2, &TopicController::cbCmd, this, ros::TransportHints().tcpNoDelay());
}

void TopicController::cbCmd(const geometry_msgs::Pose::ConstPtr &cmd) {
  Affine3d Tw;
  tf2::fromMsg(*cmd, Tw);
  Affine3d Tb = T_base_root.inverse() * Tw;

  std::lock_guard<std::mutex> lock(mtx);
  tf::transformEigenToKDL(Tb, _des_eff_pose);

  static ros::Time prev_time = ros::Time::now();
  static KDL::Frame prev_des_eff_pose = _des_eff_pose;

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - prev_time).toSec();

  if (dt < 0.01)
    return;

  getVelocity(_des_eff_pose, prev_des_eff_pose, dt, _des_eff_vel);  // TODO: get this velocity in periodic loop using Kalman filter

  if (!flagEffPose)
    flagEffPose = true;

  prev_time = current_time;
  prev_des_eff_pose = _des_eff_pose;

  if (dt > 1.0)
    _disable = true;
  else
    _disable = false;
}