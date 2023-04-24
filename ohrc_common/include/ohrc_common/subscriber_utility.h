#ifndef SUBSCRIBER_UTILITY_H
#define SUBSCRIBER_UTILITY_H

#include <ros/ros.h>

#include <mutex>

namespace subscriber_utility {

inline bool checkTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name) {
  std::vector<bool> VecSubFlag(VecSubFlagPtr.size());
  ROS_INFO_STREAM(name << ": Waiting for subscribing " << VecSubFlag.size() << " topics ...");

  while (ros::ok()) {
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

      ROS_INFO_STREAM(name << ": All topics have been subscribed.");
      return true;
    }

    ros::Duration(0.1).sleep();
  }
  return false;
}

inline bool checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx) {
  return checkTopic(VecSubFlagPtr, mtx, ros::this_node::getName());
}

inline bool checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string robot_ns) {
  return checkTopic(VecSubFlagPtr, mtx, ros::this_node::getName() + "::" + robot_ns);
}

inline bool checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name, std::string robot_ns) {
  return checkTopic(VecSubFlagPtr, mtx, name + "::" + robot_ns);
}

};  // namespace subscriber_utility

#endif  // SUBSCRIBER_UTILITY_H
