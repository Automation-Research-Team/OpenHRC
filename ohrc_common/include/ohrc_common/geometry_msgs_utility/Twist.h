#ifndef TWIST_UTILITY_H
#define TWIST_UTILITY_H

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// eigen3
#include <Eigen/Dense>
using namespace Eigen;

#include <tf2_eigen/tf2_eigen.hpp>

namespace tf2 {

inline Eigen::VectorXd fromMsg(const geometry_msgs::msg::Twist twist, Eigen::VectorXd &vector) {
  vector.resize(6);
  vector(0) = twist.linear.x;
  vector(1) = twist.linear.y;
  vector(2) = twist.linear.z;
  vector(3) = twist.angular.x;
  vector(4) = twist.angular.y;
  vector(5) = twist.angular.z;

  return vector;
}

}  // namespace tf2

namespace geometry_msgs_utility {

geometry_msgs::msg::Twist checkNanInf(geometry_msgs::msg::Twist cmd);
geometry_msgs::msg::Twist cropMinMax(geometry_msgs::msg::Twist cmd, geometry_msgs::msg::Twist min, geometry_msgs::msg::Twist max);

inline Eigen::VectorXd toVector(geometry_msgs::msg::Twist twist, Eigen::VectorXd &vector) {
  vector(0) = twist.linear.x;
  vector(1) = twist.linear.y;
  vector(2) = twist.linear.z;
  vector(3) = twist.angular.x;
  vector(4) = twist.angular.y;
  vector(5) = twist.angular.z;

  return vector;
}

inline Eigen::VectorXd toVector(geometry_msgs::msg::Twist twist) {
  Eigen::VectorXd vector(6);
  return toVector(twist, vector);
}

inline geometry_msgs::msg::Twist fromVector(Eigen::VectorXd vector, geometry_msgs::msg::Twist &twist) {
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
