#ifndef SUBSCRIBER_UTILITY_H
#define SUBSCRIBER_UTILITY_H

#include <ros/ros.h>

#include <mutex>

namespace subscriber_utility {

bool checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx);
bool checkSubTopic(std::vector<bool *> VecSubFlagPtr, std::mutex *mtx, std::string name);
};  // namespace subscriber_utility

#endif  // SUBSCRIBER_UTILITY_H
