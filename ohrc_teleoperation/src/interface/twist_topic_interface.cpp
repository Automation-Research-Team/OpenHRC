#include "ohrc_teleoperation/twist_topic_interface.hpp"

void TwistTopicInterface::initInterface() {
  n.param("trans_ratio", k_trans, 1.0);
  ROS_INFO_STREAM("translation ratio: " << k_trans);

  // stateFrameId = root_frame;

  n.getParam("topic_name", stateTopicName);
  n.getParam("frame_id", stateFrameId);

  setSubscriber();

  Affine3d T_state_base = controller->getTransform_base(this->stateFrameId);
  R = T_state_base.rotation().transpose();

  bool diablePoseFeedback;
  n.param("diable_pose_feedback", diablePoseFeedback, false);

  if (diablePoseFeedback) {
    ROS_WARN_STREAM("Pose feedback is disabled");
    controller->disablePoseFeedback();
  }
}

void TwistTopicInterface::setSubscriber() {
  subTwist = n.subscribe<geometry_msgs::Twist>(stateTopicName, 2, &TwistTopicInterface::cbTwist, this, th);
}

void TwistTopicInterface::cbTwist(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_topic);
  _twist = *msg;
  _flagTopic = true;
}

void TwistTopicInterface::setPoseFromTwistMsg(const geometry_msgs::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist) {
  if (isFirst) {
    state.pose = tf2::toMsg(controller->getT_init());
    isFirst = false;

    controller->startOperation();
  }

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
  tf::transformEigenToKDL(T, pos);
  tf::twistEigenToKDL(v, twist);
}

void TwistTopicInterface::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  geometry_msgs::Twist twist_msg;
  {
    std::lock_guard<std::mutex> lock(mtx_topic);
    if (!_flagTopic)
      return;
    twist_msg = _twist;
  }

  setPoseFromTwistMsg(twist_msg, pos, twist);
}

void TwistTopicInterface::resetInterface() {
  ROS_WARN_STREAM("Reset interface");
  state = ohrc_msgs::State();
  isFirst = true;
}