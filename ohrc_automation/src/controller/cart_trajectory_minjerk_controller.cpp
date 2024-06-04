#include "ohrc_automation/cart_trajectory_minjerk_controller.hpp"

moveit_msgs::CartesianTrajectory CartTrajectoryMinjerkController::interpolateTrajectory(const moveit_msgs::CartesianTrajectory& trj) {
  // tentative impelemntation

  int n_t = (trj.points[1].time_from_start.toSec() - trj.points[0].time_from_start.toSec()) / dt + 1;

  moveit_msgs::CartesianTrajectory trj_;
  trj_.points.resize(n_t);
  for (int i = 0; i < n_t; i++) {
    double t = i * dt;
    double s = t / (trj.points[1].time_from_start.toSec() - trj.points[0].time_from_start.toSec());
    double k = 6.0 * s * s * s * s * s - 15.0 * s * s * s * s + 10.0 * s * s * s;
    trj_.points[i].point.pose.position.x = trj.points[0].point.pose.position.x + (trj.points[1].point.pose.position.x - trj.points[0].point.pose.position.x) * k;
    trj_.points[i].point.pose.position.y = trj.points[0].point.pose.position.y + (trj.points[1].point.pose.position.y - trj.points[0].point.pose.position.y) * k;
    trj_.points[i].point.pose.position.z = trj.points[0].point.pose.position.z + (trj.points[1].point.pose.position.z - trj.points[0].point.pose.position.z) * k;
    trj_.points[i].time_from_start = ros::Duration(i * dt);
  }

  return trj_;
}