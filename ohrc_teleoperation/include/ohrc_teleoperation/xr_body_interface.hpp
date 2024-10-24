#ifndef XR_BODY_INTERFACE_HPP
#define XR_BODY_INTERFACE_HPP

#include <std_msgs/msg/float32.hpp>

#include "ohrc_msgs/msg/body_state.hpp"
#include "ohrc_teleoperation/state_topic_interface.hpp"

class XrBodyInterface : virtual public StateTopicInterface {
  rclcpp::Subscription<ohrc_msgs::msg::BodyState>::SharedPtr subBody;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubFeedback;

  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) override;

  ohrc_msgs::msg::State state;

  ohrc_msgs::msg::BodyState _body;
  enum class BodyPart { RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS, None } bodyPart;
  enum class Hand { RIGHT, LEFT } hand;

  void cbBody(const ohrc_msgs::msg::BodyState::SharedPtr msg);
  void setSubscriber() override;

protected:
  virtual bool getEnableFlag(const ohrc_msgs::msg::BodyPartState& handState, const ohrc_msgs::msg::BodyPartState& anotherBodyPartState);

public:
  using StateTopicInterface::StateTopicInterface;
  void initInterface() override;
  void resetInterface() override;
  void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
};

#endif  // XR_BODY_INTERFACE_HPP