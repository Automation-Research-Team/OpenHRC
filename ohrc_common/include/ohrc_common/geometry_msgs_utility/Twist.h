#ifndef TWIST_UTILITY_H
#define TWIST_UTILITY_H

#include <ros/ros.h>

// eigen3
#include <Eigen/Dense>
using namespace Eigen;

#include <tf2_eigen/tf2_eigen.h>

namespace geometry_msgs_utility {

geometry_msgs::Twist checkNanInf(geometry_msgs::Twist cmd);
geometry_msgs::Twist cropMinMax(geometry_msgs::Twist cmd, geometry_msgs::Twist min, geometry_msgs::Twist max);

inline Eigen::VectorXd toVector(geometry_msgs::Twist twist, Eigen::VectorXd &vector) {
  vector(0) = twist.linear.x;
  vector(1) = twist.linear.y;
  vector(2) = twist.linear.z;
  vector(3) = twist.angular.x;
  vector(4) = twist.angular.y;
  vector(5) = twist.angular.z;

  return vector;
}

inline Eigen::VectorXd toVector(geometry_msgs::Twist twist) {
  Eigen::VectorXd vector(6);
  return toVector(twist, vector);
}

inline geometry_msgs::Twist fromVector(Eigen::VectorXd vector, geometry_msgs::Twist &twist) {
  twist.linear.x = vector(0);
  twist.linear.y = vector(1);
  twist.linear.z = vector(2);
  twist.angular.x = vector(3);
  twist.angular.y = vector(4);
  twist.angular.z = vector(5);

  return twist;
}
}  // namespace geometry_msgs_utility

#endif  // TWIST_UTILITY_H
