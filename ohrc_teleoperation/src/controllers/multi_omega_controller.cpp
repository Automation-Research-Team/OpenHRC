#include "ohrc_control/multi_omega_controller.hpp"

MultiOmegaController::MultiOmegaController() {
  ros::NodeHandle n("~");

  std::string omega, haptic;
  n.param("omega_type", omega, std::string("left"));
  n.param("haptic_type", haptic, std::string("None"));
  ROS_INFO_STREAM("Omega type: " << omega << ", Haptic type: " << haptic);

  subOmega = nh.subscribe<omega_haptic_device::Omega>("/omega_driver/" + omega + "/state", 2, &MultiOmegaController::cbOmegaState, this, th);
  pubOmegaForce = nh.advertise<geometry_msgs::Wrench>("/omega_driver/" + omega + "/cmd_force", 2);
  pubOmegaForceVis = nh.advertise<geometry_msgs::WrenchStamped>("cmd_force_vis", 2);

  // subLeaderEnergy = nh.subscribe<std_msgs::Float32>("/passivity_observer/leader/energy", 2, &OmegaController::cbEnergy, this, th);
  // pubEnergy = nh.advertise<std_msgs::Float32>("/passivity_observer/follower/energy", 2);

  if (haptic == "PositionPositionFeedback")
    hapticType = HapticType::PositionPositionFeedback;
  else if (haptic == "PositionForce")
    hapticType = HapticType::PositionForce;
  else if (haptic == "PositionForceFeedback")
    hapticType = HapticType::PositionForceFeedback;
  else
    hapticType = HapticType::None;
}

void MultiOmegaController::cbOmegaState(const omega_haptic_device::Omega::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_omega);
  _omegaState = *msg;
}

void MultiOmegaController::starting() {
  MultiCartController::starting();

  T_omega_base.resize(nRobot);
  s_updateManualTargetPoses.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    T_omega_base[i] = cartControllers[i]->getTransform_base("omega_link");
}

void MultiOmegaController::updateManualTargetPose(KDL::Frame& pos, KDL::Twist& twist, CartController* controller) {
  omega_haptic_device::Omega omegaState;
  {
    std::lock_guard<std::mutex> lock(mtx_omega);
    omegaState = _omegaState;
  }

  double k_trans = 2.0;  // position slacing factor
  Matrix3d R = T_omega_base[controller->getIndex()].rotation().transpose();
  Affine3d T_omega_omega;
  tf2::fromMsg(omegaState.pose, T_omega_omega);
  Matrix<double, 6, 1> v_omega_omega;
  tf2::fromMsg(omegaState.twist, v_omega_omega);

  Affine3d T_omega = Translation3d(k_trans * R * T_omega_omega.translation()) * (R * T_omega_omega.rotation());
  VectorXd v_omega = (VectorXd(6) << k_trans * R * v_omega_omega.head(3), R * v_omega_omega.tail(3)).finished();

  // Affine3d T_omega = Translation3d(k_trans * R * Vector3d(-omageState.pose.position.x, -omageState.pose.position.y, omageState.pose.position.z)) *
  //                    (R * Quaterniond(omageState.pose.orientation.w, -omageState.pose.orientation.x, -omageState.pose.orientation.y, omageState.pose.orientation.z));
  // VectorXd v_omega = (VectorXd(6) << k_trans * R * Vector3d(-omageState.twist.linear.x, -omageState.twist.linear.y, omageState.twist.linear.z),
  //                     R * Vector3d(-omageState.twist.angular.x, -omageState.twist.angular.y, omageState.twist.angular.z))
  //                        .finished();

  s_updateManualTargetPose* s = &this->s_updateManualTargetPoses[controller->getIndex()];
  if (s->isFirst) {
    s->T = controller->getT_init();
    s->T_start = controller->getT_init();
    s->T_omega_start = T_omega;
    s->isFirst = false;
  }

  // Affine3d T = s.T, T_start = s.T_start, T_omega_start = s.T_omega_start;

  VectorXd v = VectorXd::Zero(6);

  if (omegaState.gripper.button) {
    s->T = Translation3d(T_omega.translation() - s->T_omega_start.translation() + s->T_start.translation()) * (T_omega.rotation() * controller->getT_init().rotation() * R);  // TODO: correct???
    v = v_omega;
    controller->enableOperation();
  } else {
    s->T_start = s->T;
    s->T_omega_start = T_omega;
    controller->disableOperation();
    // v = 0.9;  // TODO: stop smoothly
  }

  tf::transformEigenToKDL(s->T, pos);
  tf::twistEigenToKDL(v, twist);
  // update pos and twist
}

#if 0
void MultiOmegaController::updateAutoTargetPose(KDL::Frame& pos, KDL::Twist& twist, CartController* controller) {
  KDL::Frame pos_d;
  pos_d.p = pos.p + KDL::Vector(-0.15, -0.5, 0.0);
  static ros::Time t0 = ros::Time::now();
  double T = 10.0;
  double s = (ros::Time::now() - t0).toSec() / T, s2, s3, s4, s5;
  if (s > 1.0)
    s = 1.0;
  s2 = s * s;
  s3 = s2 * s;
  s4 = s3 * s;
  s5 = s4 * s;

  // min jerk trajectory
  pos.p = pos.p + (pos_d.p - pos.p) * (6.0 * s5 - 15.0 * s4 + 10.0 * s3);
  twist.vel = (pos_d.p - pos.p) * (30.0 * s4 - 60.0 * s3 + 30.0 * s2) / T;
}
#endif

void MultiOmegaController::feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, CartController* controller) {
  VectorXd ft_feedback = VectorXd::Zero(6);
  double k_force = 0.0;
  if (hapticType == HapticType::PositionPositionFeedback) {
    k_force = 50.0;
    ft_feedback.head(3) = k_force * (T_cur.translation() - T_des.translation());
  }

  VectorXd ft_feedback_vis = ft_feedback;
  ft_feedback_vis.head(3) = T_cur.rotation() * ft_feedback_vis.head(3);
  ft_feedback_vis.tail(3) = T_cur.rotation() * ft_feedback_vis.tail(3);
  // ft_feedback[0] = 10.0;
  geometry_msgs::WrenchStamped wrench_omega_vis;
  wrench_omega_vis.header.stamp = ros::Time::now();
  wrench_omega_vis.header.frame_id = controller->getRobotNs() + controller->getChainEnd();
  wrench_omega_vis.wrench = tf2::toMsg(ft_feedback_vis, wrench_omega_vis.wrench);
  pubOmegaForceVis.publish(wrench_omega_vis);

  ft_feedback.head(3) = T_omega_base[controller->getIndex()].rotation() * ft_feedback.head(3);
  ft_feedback.tail(3) = T_omega_base[controller->getIndex()].rotation() * ft_feedback.tail(3);
  geometry_msgs::Wrench wrench_omega;
  wrench_omega = tf2::toMsg(ft_feedback, wrench_omega);

  if (controller->getOperationEnable())
    pubOmegaForce.publish(wrench_omega);
  else
    pubOmegaForce.publish(geometry_msgs::Wrench());
}