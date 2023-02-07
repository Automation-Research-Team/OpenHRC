#ifndef JOY_TOPIC_ITNERFACE_HPP
#define JOY_TOPIC_ITNERFACE_HPP

#include "ohrc_teleoperation/twist_topic_interface.hpp"
#include "sensor_msgs/Joy.h"

class JoyTopicInterface : public TwistTopicInterface {
protected:
  ros::Subscriber subJoy;

  sensor_msgs::Joy _joy;
  double gain_h = 0.1, gain_r = 0.1;

  std::string stateTopicName = "/spacenav/joy", stateFrameId = "world";

  void cbJoy(const sensor_msgs::Joy::ConstPtr& msg);

  void setSubscriber() override;

public:
  using TwistTopicInterface::TwistTopicInterface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  // void resetInterface() override;
};

#endif  // JOY_TOPIC_ITNERFACE_HPP