#include "ohrc_teleoperation/twist_topic_interface.hpp"

void TwistTopicInterface::initInterface() {
  // n.param("trans_ratio", k_trans, 1.0);
  RclcppUtility::declare_and_get_parameter(node, "trans_ratio", 1.0, k_trans);
  // RCL("translation ratio: " << k_trans);

  setSubscriber();

  Affine3d T_state_base = controller->getTransform_base(this->stateFrameId);
  R = T_state_base.rotation().transpose();

  bool diablePoseFeedback;
  // n.param("diable_pose_feedback", diablePoseFeedback, false);
  RclcppUtility::declare_and_get_parameter(node, "diable_pose_feedback", false, diablePoseFeedback);

  if (diablePoseFeedback) {
    // ROS_WARN_STREAM("Pose feedback is disabled");
    RCLCPP_WARN_STREAM(node->get_logger(), "Pose feedback is disabled");
    controller->disablePoseFeedback();
  }
}

void TwistTopicInterface::setSubscriber() {
  this->getTopicAndFrameName("/cmd_vel", "user_frame");
  // subTwist = n.subscribe<geometry_msgs::msg::Twist>(stateTopicName, 2, &TwistTopicInterface::cbTwist, this, th);
  subTwist = node->create_subscription<geometry_msgs::msg::Twist>(stateTopicName, rclcpp::QoS(1), std::bind(&TwistTopicInterface::cbTwist, this, std::placeholders::_1));
}

void TwistTopicInterface::cbTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_topic);
  _twist = *msg;
  _flagTopic = true;
}

void TwistTopicInterface::setPoseFromTwistMsg(const geometry_msgs::msg::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist) {
  if (isFirst) {
    state.pose = tf2::toMsg(controller->getT_init());
    isFirst = false;

    controller->startOperation();
  }

  ohrc_msgs::msg::State state = this->state;
  state.enabled = true;
  state.twist = twist_msg;
  state.pose.position.x += state.twist.linear.x * dt;
  state.pose.position.y += state.twist.linear.y * dt;
  state.pose.position.z += state.twist.linear.z * dt;

  Quaterniond quat;
  tf2::fromMsg(state.pose.orientation, quat);

  Vector3d omega;
  tf2::fromMsg(state.twist.angular, omega);

  if (omega.norm() > 1.0e-6)  // only if omega != 0, quaternion is integrated. Otherwise, keep its previous values
    state.pose.orientation = tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(omega.norm() * dt, omega / omega.norm())) * quat);

  Affine3d T_state_state;
  tf2::fromMsg(state.pose, T_state_state);
  Matrix<double, 6, 1> v_state_state;
  tf2::fromMsg(state.twist, v_state_state);

  Affine3d T = Translation3d(k_trans * R * T_state_state.translation()) * (R * T_state_state.rotation());
  VectorXd v = (VectorXd(6) << k_trans * R * v_state_state.head(3), R * v_state_state.tail(3)).finished();

  // update pos and twist
  tf2::transformEigenToKDL(T, pos);
  tf2::twistEigenToKDL(v, twist);

  this->state = state;
}

void TwistTopicInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pos, KDL::Twist& twist) {
  geometry_msgs::msg::Twist twist_msg;
  {
    std::lock_guard<std::mutex> lock(mtx_topic);
    if (!_flagTopic)
      return;
    twist_msg = _twist;
  }

  setPoseFromTwistMsg(twist_msg, pos, twist);
}

void TwistTopicInterface::resetInterface() {
  RCLCPP_INFO_STREAM(node->get_logger(), "Reset interface");
  state = ohrc_msgs::msg::State();
  isFirst = true;
}