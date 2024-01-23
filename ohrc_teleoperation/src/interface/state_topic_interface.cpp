#include "ohrc_teleoperation/state_topic_interface.hpp"

void StateTopicInterface::initInterface() {
  n.param("trans_ratio", k_trans, 1.0);
  ROS_INFO_STREAM("translation ratio: " << k_trans);

  setSubscriber();

  T_state_base = controller->getTransform_base(this->stateFrameId);
}

void StateTopicInterface::setSubscriber() {
  subState = n.subscribe<ohrc_msgs::State>(stateTopicName, 2, &StateTopicInterface::cbState, this, th);
}

void StateTopicInterface::cbState(const ohrc_msgs::State::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  _state = *msg;
}

void StateTopicInterface::getTargetState(const ohrc_msgs::State& state, KDL::Frame& pos, KDL::Twist& twist) {
  // double k_trans = 2.0;  // position slacing factor
  Matrix3d R = T_state_base.rotation();
  Affine3d T_state_state;
  tf2::fromMsg(state.pose, T_state_state);
  Matrix<double, 6, 1> v_state_state;
  tf2::fromMsg(state.twist, v_state_state);

  Affine3d T_state = Translation3d(k_trans * R * T_state_state.translation()) * (R * T_state_state.rotation());
  VectorXd v_state = (VectorXd(6) << k_trans * R * v_state_state.head(3), R * v_state_state.tail(3)).finished();

  if (isFirst && !state.enabled)
    return;

  if (isFirst) {
    T = controller->getT_init();
    T_start = controller->getT_init();
    T_state_start = T_state;
    isFirst = false;
  }

  VectorXd v = VectorXd::Zero(6);
  if (state.enabled) {
    T = Translation3d(T_state.translation() - T_state_start.translation() + T_start.translation()) *
        (T_state.rotation() * controller->getT_init().rotation() * R.transpose());  // TODO: correct???
    v = v_state;
    controller->enableOperation();
  } else {
    T_start = T;
    T_state_start = T_state;
    controller->disableOperation();
    // v = 0.9;  // TODO: stop smoothly
  }

  tf::transformEigenToKDL(T, pos);
  tf::twistEigenToKDL(v, twist);

  if (state.reset)
    this->reset();
  // update pos and twist
}

void StateTopicInterface::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  ohrc_msgs::State state;
  {
    std::lock_guard<std::mutex> lock(mtx_state);
    state = _state;
  }
  modifyTargetState(state);

  getTargetState(state, pos, twist);
}
