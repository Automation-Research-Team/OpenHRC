#include "ohrc_teleoperation/joy_topic_interface.hpp"

void JoyTopicInterface::initInterface() {
  TwistTopicInterface::initInterface();
  n.getParam("gain/horizontal", gain_h);
  n.getParam("gain/rotational", gain_r);
}

void JoyTopicInterface::setSubscriber() {
  subJoy = n.subscribe<sensor_msgs::Joy>(stateTopicName, 2, &JoyTopicInterface::cbJoy, this, th);
}

void JoyTopicInterface::cbJoy(const sensor_msgs::Joy::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_topic);
  _joy = *msg;
  _flagTopic = true;
}

void JoyTopicInterface::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  sensor_msgs::Joy joy;
  {
    std::lock_guard<std::mutex> lock(mtx_topic);
    joy = _joy;
    if (!_flagTopic)
      return;
  }

  if (joy.axes.size() < 6 || joy.buttons.size() < 2)
    return;

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = joy.axes[0] * gain_h;
  twist_msg.linear.y = joy.axes[1] * gain_h;
  twist_msg.linear.z = joy.axes[2] * gain_h;
  twist_msg.angular.x = joy.axes[3] * gain_r;
  twist_msg.angular.y = joy.axes[4] * gain_r;
  twist_msg.angular.z = joy.axes[5] * gain_r;

  if (joy.buttons[1] == 1.0) {
    controller->resetPose();
    resetInterface();
  }
  setPoseFromTwistMsg(twist_msg, pos, twist);
}

// void JoyTopicInterface::resetInterface() {
//   ROS_WARN_STREAM("Reset marker position");
//   state = ohrc_msgs::State();
// }