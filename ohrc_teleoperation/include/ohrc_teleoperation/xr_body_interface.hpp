#ifndef XR_BODY_INTERFACE_HPP
#define XR_BODY_INTERFACE_HPP

#include <std_msgs/Float32.h>

#include "ohrc_msgs/BodyState.h"
#include "ohrc_teleoperation/state_topic_interface.hpp"

class XrBodyInterface : virtual public StateTopicInterface {
  ros::Subscriber subBody;
  ros::Publisher pubFeedback;

  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) override;

  ohrc_msgs::State state;

  ohrc_msgs::BodyState _body;
  enum class BodyPart { RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS, NONE } bodyPart;
  enum class Hand { RIGHT, LEFT } hand;

  void cbBody(const ohrc_msgs::BodyState::ConstPtr& msg);
  void setSubscriber() override;

protected:
  virtual bool getEnableFlag(const ohrc_msgs::HandState& handState, const ohrc_msgs::HandState& anotherHandState);

public:
  using StateTopicInterface::StateTopicInterface;
  void initInterface() override;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
};

#endif  // XR_BODY_INTERFACE_HPP