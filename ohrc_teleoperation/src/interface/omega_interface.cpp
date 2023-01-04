#include "ohrc_teleoperation/omega_interface.hpp"

OmegaInterface::OmegaInterface() {
  ros::NodeHandle n("~");

  std::string omega, haptic;
  n.param("omega_type", omega, std::string("left"));
  n.param("haptic_type", haptic, std::string("None"));
  ROS_INFO_STREAM("Omega type: " << omega << ", Haptic type: " << haptic);

  stateTopicName = "/omega_driver/" + omega + "/state";
  stateFrameId = "omega_link";

  pubOmegaForce = nh.advertise<geometry_msgs::Wrench>("/omega_driver/" + omega + "/cmd_force", 2);
  pubOmegaForceVis = nh.advertise<geometry_msgs::WrenchStamped>("cmd_force_vis", 2);

  hapticType = magic_enum::enum_cast<HapticType>(haptic).value_or(HapticType::None);
}

void OmegaInterface::feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, CartController* controller) {
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

  ft_feedback.head(3) = T_state_base[controller->getIndex()].rotation() * ft_feedback.head(3);
  ft_feedback.tail(3) = T_state_base[controller->getIndex()].rotation() * ft_feedback.tail(3);
  geometry_msgs::Wrench wrench_omega;
  wrench_omega = tf2::toMsg(ft_feedback, wrench_omega);

  if (controller->getOperationEnable())
    pubOmegaForce.publish(wrench_omega);
  else
    pubOmegaForce.publish(geometry_msgs::Wrench());
}