/**
 * @file Twist.cpp
 * @author Shunki Itadera
 * @date June. 2021
 * @brief utility library for Point message
 **/

#include "ohrc_common/geometry_msgs_utility/Twist.h"

namespace geometry_msgs_utility {
geometry_msgs::msg::Twist checkNanInf(geometry_msgs::msg::Twist cmd) {
  bool error = false;
  if (std::isnan(cmd.linear.x) || std::isnan(cmd.linear.y) || std::isnan(cmd.angular.z)) {
    // ROS_ERROR("cmd_vel contains Nan value!");
    error = true;
  }

  if (std::isinf(cmd.linear.x) || std::isinf(cmd.linear.y) || std::isinf(cmd.angular.z)) {
    // ROS_ERROR("cmd_vel contains Inf value!");
    error = true;
  }

  if (error) {
    rclcpp::shutdown();
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
  }

  return cmd;
}

geometry_msgs::msg::Twist cropMinMax(geometry_msgs::msg::Twist cmd, geometry_msgs::msg::Twist min, geometry_msgs::msg::Twist max) {
  Matrix<double, 6, 1> cmd_v, min_v, max_v;
  fromMsg(cmd, cmd_v);
  fromMsg(min, min_v);
  fromMsg(max, max_v);

  for (int i = 0; i < 6; i++) {
    if (cmd_v(i) < min_v(i))
      cmd_v(i) = min_v(i);

    if (cmd_v(i) > max_v(i))
      cmd_v(i) = max_v(i);
  }

  return toMsg(cmd_v);
}

};  // namespace geometry_msgs_utility