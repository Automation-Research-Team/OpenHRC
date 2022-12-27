#ifndef ODOMETRY_UTILITY_H
#define ODOMETRY_UTILITY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// tf2
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

namespace nav_msgs_utility {
class Odometry {

public:
  static void toVector_3dof(nav_msgs::Odometry odom, Eigen::MatrixXd &vector);
  static void toVector_6dof(nav_msgs::Odometry odom, Eigen::MatrixXd &vector);

  static void fromVector_3dof(Eigen::MatrixXd vector, nav_msgs::Odometry &odom);
  static void fromVector_6dof(Eigen::MatrixXd vector, nav_msgs::Odometry &odom);

  static void transFromOdom(nav_msgs::Odometry odom, geometry_msgs::TransformStamped &trans_base_odom);
  static void transToOdom(nav_msgs::Odometry odom, geometry_msgs::TransformStamped &trans_odom_base);
};
} // namespace nav_msgs_utility

#endif // ODOMETRY_UTILITY_H
