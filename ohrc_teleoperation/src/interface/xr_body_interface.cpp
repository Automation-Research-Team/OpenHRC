#include "ohrc_teleoperation/xr_body_interface.hpp"

void XrBodyInterface::initInterface() {
  stateTopicName = "/body_state";
  stateFrameId = "world";

  StateTopicInterface::initInterface();
}

void XrBodyInterface::setSubscriber() {
  subBody = n.subscribe<ohrc_msgs::BodyState>(stateTopicName, 2, &XrBodyInterface::cbBody, this, th);
}

void XrBodyInterface::cbBody(const ohrc_msgs::BodyState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  _body = *msg;
  _flagTopic = true;
}

void XrBodyInterface::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  ohrc_msgs::BodyState body;
  {
    std::lock_guard<std::mutex> lock(mtx_state);
    body = _body;
    if (!_flagTopic)
      return;
  }

  ohrc_msgs::State state;
  state.pose = body.right_hand.pose;
  if (body.right_hand.index_trigger > 0.9)
    state.enabled = true;
  else
    state.enabled = false;

  // ROS_INFO_STREAM(state);
  getTargetState(state, pose, twist);
}