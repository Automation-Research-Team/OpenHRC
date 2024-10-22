#ifndef SUBSCRIBER_UTILITY_H
#define SUBSCRIBER_UTILITY_H

// #include <ros/ros.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

namespace subscriber_utility {

inline bool checkTopic(rclcpp::Node::SharedPtr node, std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name) {
  std::vector<bool> VecSubFlag(VecSubFlagPtr.size());
  RCLCPP_INFO_STREAM(node->get_logger(), name << ": Waiting for subscribing " << VecSubFlag.size() << " topics ...");
  rclcpp::spin_some(node);

  while (rclcpp::ok()) {
    mtx->lock();
    for (int i = 0; i < VecSubFlag.size(); ++i)
      VecSubFlag[i] = *VecSubFlagPtr[i];
    mtx->unlock();

    // check topics
    if (std::find(VecSubFlag.begin(), VecSubFlag.end(), false) == VecSubFlag.end()) {
      mtx->lock();
      for (int i = 0; i < VecSubFlag.size(); ++i)
        *VecSubFlagPtr[i] = false;
      mtx->unlock();

      RCLCPP_INFO_STREAM(node->get_logger(), name << ": All topics have been subscribed.");
      return true;
    }

    // rclcpp::spin_some(node);

    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

inline bool checkSubTopic(rclcpp::Node::SharedPtr node, std::vector<bool *> VecSubFlagPtr, std::mutex *mtx) {
  return checkTopic(node, VecSubFlagPtr, mtx, "");
}

inline bool checkSubTopic(rclcpp::Node::SharedPtr node, std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string robot_ns) {
  return checkTopic(node, VecSubFlagPtr, mtx, robot_ns);
}

inline bool checkSubTopic(rclcpp::Node::SharedPtr node, std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name, std::string robot_ns) {
  return checkTopic(node, VecSubFlagPtr, mtx, name + "::" + robot_ns);
}

};  // namespace subscriber_utility

#endif  // SUBSCRIBER_UTILITY_H
