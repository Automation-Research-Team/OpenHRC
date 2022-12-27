#include "ohrc_common/subscriber_utility.h"

bool subscriber_utility::checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name) {
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

bool subscriber_utility::checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx) {
  return checkSubTopic(VecSubFlagPtr, mtx, ros::this_node::getName());
}