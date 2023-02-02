#include "ohrc_teleoperation/twist_topic_interface.hpp"

TwistTopicInterface::TwistTopicInterface() {
  ros::NodeHandle n("~");
  n.param("trans_ratio", k_trans, 1.0);
  ROS_INFO_STREAM("translation ratio: " << k_trans);

  stateFrameId = root_frame;
}

void TwistTopicInterface::cbTwist(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_twist);
  _twist = *msg;
}

void TwistTopicInterface::starting() {
  subTwist = nh.subscribe<geometry_msgs::Twist>(stateTopicName, 2, &TwistTopicInterface::cbTwist, this, th);

  MultiCartController::starting();

  T_state_base.resize(nRobot);
  s_updateManualTargetPoses.resize(nRobot);

  for (int i = 0; i < nRobot; i++)
    T_state_base[i] = cartControllers[i]->getTransform_base(this->stateFrameId);
}

void TwistTopicInterface::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist, CartController* controller) {
  geometry_msgs::Twist twist_msg;
  {
    std::lock_guard<std::mutex> lock(mtx_twist);
    twist_msg = _twist;
  }
  state.enabled = true;

  state.twist = twist_msg, state.twist;
  state.pose.position.x += state.twist.linear.x * dt;
  state.pose.position.y += state.twist.linear.y * dt;
  state.pose.position.z += state.twist.linear.z * dt;

  Quaterniond quat;
  tf2::fromMsg(state.pose.orientation, quat);

  Vector3d omega;
  tf2::fromMsg(state.twist.angular, omega);

  Vector4d dQuat;                                                      // (x,y,z,w)
  dQuat.head(3) = 0.5 * (quat.w() * omega - quat.vec().cross(omega));  // Handbook of Robotics (Siciliano, 2nd Ed.) below eq.(2.8)
  dQuat(3) = -0.5 * omega.transpose() * quat.vec();

  Vector4d newQaut = (quat.coeffs() + dQuat * dt).normalized();                                                // (x,y,z,w)
  quat = rotation_util::checkFlipQuaternionSign(Quaterniond(newQaut(3), newQaut(0), newQaut(1), newQaut(2)));  // Quaterniond(w, x, y, z)

  state.pose.orientation = tf2::toMsg(quat);

  // double k_trans = 2.0;  // position slacing factor
  Matrix3d R = T_state_base[controller->getIndex()].rotation().transpose();
  Affine3d T_state_state;
  tf2::fromMsg(state.pose, T_state_state);
  Matrix<double, 6, 1> v_state_state;
  tf2::fromMsg(state.twist, v_state_state);

  Affine3d T_state = Translation3d(k_trans * R * T_state_state.translation()) * (R * T_state_state.rotation());
  VectorXd v_state = (VectorXd(6) << k_trans * R * v_state_state.head(3), R * v_state_state.tail(3)).finished();

  s_updateManualTargetPose* s = &this->s_updateManualTargetPoses[controller->getIndex()];
  if (s->isFirst) {
    s->T = controller->getT_init();
    s->T_start = controller->getT_init();
    s->T_state_start = T_state;
    s->isFirst = false;
  }

  VectorXd v = VectorXd::Zero(6);
  if (state.enabled) {
    s->T = Translation3d(T_state.translation() - s->T_state_start.translation() + s->T_start.translation()) *
           (T_state.rotation() * controller->getT_init().rotation() * R);  // TODO: correct???
    v = v_state;
    controller->enableOperation();
  } else {
    s->T_start = s->T;
    s->T_state_start = T_state;
    controller->disableOperation();
    // v = 0.9;  // TODO: stop smoothly
  }

  tf::transformEigenToKDL(s->T, pos);
  tf::twistEigenToKDL(v, twist);
  // update pos and twist
}

void TwistTopicInterface::resetInterface() {
  ROS_WARN_STREAM("Reset marker position");
  state = ohrc_msgs::State();
}