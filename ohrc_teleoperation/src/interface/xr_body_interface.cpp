#include "ohrc_teleoperation/xr_body_interface.hpp"

void XrBodyInterface::initInterface() {
  stateTopicName = "/body_state";
  stateFrameId = "xr_frame";

  StateTopicInterface::initInterface();

  // bodyPart = BodyPart::EITHER_HANDS;

  std::string bodyPart_str;
  if (!n.param(controller->getRobotNs() + "body_part", bodyPart_str, std::string("RIGHT_HAND")))
    ROS_WARN_STREAM("Used body part is not choisen {RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS}: Default RIGHT_HAND");
  else
    ROS_INFO_STREAM("body_part: " << bodyPart_str);

  bodyPart = magic_enum::enum_cast<BodyPart>(bodyPart_str).value_or(BodyPart::NONE);
  if (bodyPart == BodyPart::NONE) {
    ROS_FATAL("BodyPart is not correctly choisen from {RIGHT_HAND, LEFT_HAND, HEAD, EITHER_HANDS}");
    return;
  }

  controller->updateFilterCutoff(5.0, 10.0);

  pubFeedback = n.advertise<std_msgs::Float32>("/feedback/"+bodyPart_str, 2);
}

void XrBodyInterface::setSubscriber() {
  subBody = n.subscribe<ohrc_msgs::BodyState>(stateTopicName, 2, &XrBodyInterface::cbBody, this, th);
}

void XrBodyInterface::cbBody(const ohrc_msgs::BodyState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  _body = *msg;
  _flagTopic = true;
}

bool XrBodyInterface::getEnableFlag(const ohrc_msgs::HandState& handState, const ohrc_msgs::HandState& anotherHandState) {
  if (handState.grip > 0.9)
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
      if (getEnableFlag(body.right_hand, body.left_hand))
        state.enabled = true;
      break;

    case BodyPart::LEFT_HAND:
      state.pose = body.left_hand.pose;
      state.twist = body.left_hand.twist;
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
      } else {
        state.pose = body.left_hand.pose;
        state.twist = body.left_hand.twist;
      }
      break;

    default:
      break;
  }

  // ROS_INFO_STREAM(state);
  getTargetState(state, pose, twist);
  this->state = state;  // TODO: check if this is necessary
}

void XrBodyInterface::resetInterface(){
  this->isFirst = true;
}

void XrBodyInterface::feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {
  std_msgs::Float32 amp;

  amp.data = std::max(std::min((tf2::fromMsg(controller->getForceEef().wrench).head(3).norm()-1.0)/10.0, 1.0),0.0);

  if (controller->getOperationEnable())
    pubFeedback.publish(amp);
  else
    pubFeedback.publish(std_msgs::Float32());
}