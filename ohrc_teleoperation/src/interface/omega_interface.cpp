#include "ohrc_teleoperation/omega_interface.hpp"

void OmegaInterface::initInterface() {
  std::string omega, haptic;
  n.param("omega_type", omega, std::string("left"));
  n.param("haptic_type", haptic, std::string("None"));
  ROS_INFO_STREAM("Omega type: " << omega << ", Haptic type: " << haptic);

  stateTopicName = "/omega_driver/" + omega + "/state";
  stateFrameId = "omega_link";

  pubOmegaForce = n.advertise<geometry_msgs::Wrench>("/omega_driver/" + omega + "/cmd_force", 2);
  pubOmegaForceVis = n.advertise<geometry_msgs::WrenchStamped>("cmd_force_vis", 2);

  hapticType = magic_enum::enum_cast<HapticType>(haptic).value_or(HapticType::None);

  StateTopicInterface::initInterface();

  controller->updateFilterCutoff(20.0, 20.0);
}

void OmegaInterface::modifyTargetState(ohrc_msgs::State& state) {
  state.enabled = state.gripper.button;
  ft_omega = tf2::fromMsg(state.wrench);
}

// void OmegaInterface::feedbackCart(const Affine3d& T_cur, const Affine3d& T_des) {
void OmegaInterface::feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {
  Affine3d T_cur, T_des;

  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);
  tf::transformKDLToEigen(frame, T_cur);
  tf::transformKDLToEigen(targetPos, T_des);

  VectorXd ft_feedback = VectorXd::Zero(6);
  double k_force = 0.0;
  switch (hapticType) {
    case HapticType::PositionPositionFeedback:  // position-positon feedback control
      tf::transformKDLToEigen(targetPos, T_des);
      k_force = 100.0;
      ft_feedback.head(3) += k_force * (T_cur.translation() - T_des.translation());
      break;

    case HapticType::PositionForce:  // force reflection type (a.k.a. position-force control)
      k_force = 0.2;
      ft_feedback += k_force * tf2::fromMsg(controller->getForceEef().wrench).head(3);
      break;

    case HapticType::PositionForceFeedback:// force feedback type (a.k.a. position-force feedback control)
      k_force = 0.1;
      ft_feedback += k_force*(tf2::fromMsg(controller->getForceEef().wrench).head(3) - T_cur.rotation().transpose()* T_state_base.rotation()* ft_omega.head(3));  
      break;

    default:
      break;
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

  ft_feedback.head(3) = T_state_base.rotation() * ft_feedback.head(3);
  ft_feedback.tail(3) = T_state_base.rotation() * ft_feedback.tail(3);
  geometry_msgs::Wrench wrench_omega;
  wrench_omega = tf2::toMsg(ft_feedback, wrench_omega);

  if (controller->getOperationEnable())
    pubOmegaForce.publish(wrench_omega);
  else
    pubOmegaForce.publish(geometry_msgs::Wrench());
}