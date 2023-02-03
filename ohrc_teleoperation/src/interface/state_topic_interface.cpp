#include "ohrc_teleoperation/state_topic_interface.hpp"

StateTopicInterface::StateTopicInterface() {
  ros::NodeHandle n("~");
  n.param("trans_ratio", k_trans, 1.0);
  ROS_INFO_STREAM("translation ratio: " << k_trans);
}

void StateTopicInterface::cbState(const ohrc_msgs::State::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  _state = *msg;
}

void StateTopicInterface::starting() {
  subState = nh.subscribe<ohrc_msgs::State>(stateTopicName, 2, &StateTopicInterface::cbState, this, th);

  MultiCartController::starting();

  T_state_base.resize(nRobot);
  s_updateManualTargetPoses.resize(nRobot);

  for (int i = 0; i < nRobot; i++)
    T_state_base[i] = cartControllers[i]->getTransform_base(this->stateFrameId);
}

void StateTopicInterface::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist, CartController* controller) {
  ohrc_msgs::State state;
  {
    std::lock_guard<std::mutex> lock(mtx_state);
    state = _state;
  }

  modifyTargetState(state);

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
