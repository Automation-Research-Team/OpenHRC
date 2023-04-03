#ifndef XR_BODY_INTERFACE_HPP
#define XR_BODY_INTERFACE_HPP

#include "ohrc_msgs/BodyState.h"
#include "ohrc_teleoperation/state_topic_interface.hpp"

class XrBodyInterface : public StateTopicInterface {
  ros::Subscriber subBody;

  ohrc_msgs::BodyState _body;

  void cbBody(const ohrc_msgs::BodyState::ConstPtr& msg);
  void setSubscriber() override;

public:
  using StateTopicInterface::StateTopicInterface;
  void initInterface() override;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
};

#endif  // XR_BODY_INTERFACE_HPP