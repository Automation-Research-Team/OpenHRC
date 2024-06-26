#include "ohrc_teleoperation/xr_body_interface.hpp"

void XrBodyInterface::initInterface() {
  StateTopicInterface::initInterface();

  std::string bodyPart_str;
  if (!n.param(controller->getRobotNs() + "body_part", bodyPart_str, std::string("RIGHT_HAND")))
    ROS_WARN_STREAM("Used body part for " << controller->getRobotNs() << " is not choisen {RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS}: Default RIGHT_HAND");
  else
    ROS_INFO_STREAM("body_part: " << bodyPart_str);

  bodyPart = magic_enum::enum_cast<BodyPart>(bodyPart_str).value_or(BodyPart::NONE);
  if (bodyPart == BodyPart::NONE) {
    ROS_FATAL("BodyPart is not correctly choisen from {RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS}");
    return;
  }

  // controller->updateFilterCutoff(10.0, 10.0);
  controller->disablePoseFeedback();
  controller->updateVelFilterCutoff(70.0);

  pubFeedback = n.advertise<std_msgs::Float32>("/feedback/" + bodyPart_str, 2);
}

void XrBodyInterface::setSubscriber() {
  getTopicAndFrameName("/body_state", "user_frame");
  subBody = n.subscribe<ohrc_msgs::BodyState>(stateTopicName, 1, &XrBodyInterface::cbBody, this, th);
}

void XrBodyInterface::cbBody(const ohrc_msgs::BodyState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  _body = *msg;
  _flagTopic = true;
}

bool XrBodyInterface::getEnableFlag(const ohrc_msgs::BodyPartState& handState, const ohrc_msgs::BodyPartState& anotherBodyPartState) {
  if (handState.grip > 0.95)
    return true;
  else
    return false;
}

void XrBodyInterface::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  ohrc_msgs::BodyState body;
  {
    std::lock_guard<std::mutex> lock(mtx_state);
    body = _body;
    if (!_flagTopic)
      return;
  }

  ohrc_msgs::State state = this->state;
  state.enabled = false;
  switch (bodyPart) {
    case BodyPart::RIGHT_HAND:
      state.pose = body.right_hand.pose;
      state.twist = body.right_hand.twist;
      state.reset = body.right_hand.button[2];  // stick click
      if (getEnableFlag(body.right_hand, body.left_hand))
        state.enabled = true;
      break;

    case BodyPart::LEFT_HAND:
      state.pose = body.left_hand.pose;
      state.twist = body.left_hand.twist;
      state.reset = body.left_hand.button[2];  // stick click
      if (getEnableFlag(body.left_hand, body.right_hand))
        state.enabled = true;
      break;

    case BodyPart::HEAD:
      state.pose = body.head.pose;
      state.twist = body.head.twist;
      if (getEnableFlag(body.right_hand, body.left_hand) || getEnableFlag(body.left_hand, body.right_hand))
        state.enabled = true;
      break;

    case BodyPart::EITHER_HANDS:
      if (getEnableFlag(body.right_hand, body.left_hand)) {
        hand = Hand::RIGHT;
        state.enabled = true;
      } else if (getEnableFlag(body.left_hand, body.right_hand)) {
        hand = Hand::LEFT;
        state.enabled = true;
      }

      if (hand == Hand::RIGHT) {
        state.pose = body.right_hand.pose;
        state.twist = body.right_hand.twist;
        state.reset = body.right_hand.button[2];  // stick click
      } else {
        state.pose = body.left_hand.pose;
        state.twist = body.left_hand.twist;
        state.reset = body.left_hand.button[2];  // stick click
      }
      break;

    default:
      break;
  }

  // ROS_INFO_STREAM(state);
  getTargetState(state, pose, twist);
  this->state = state;  // TODO: check if this is necessary
}

void XrBodyInterface::resetInterface() {
  this->isFirst = true;
}

void XrBodyInterface::feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {
  std_msgs::Float32 amp;

  amp.data = std::max(std::min((tf2::fromMsg(controller->getForceEef().wrench).head(3).norm() - 1.0) / 10.0, 1.0), 0.0);

  if (controller->getOperationEnable())
    pubFeedback.publish(amp);
  else
    pubFeedback.publish(std_msgs::Float32());
}