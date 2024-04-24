#include "ohrc_control/feedback_controller.hpp"

void FeedbackController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd f = controller->getForceEefVec();
  VectorXd v;

  if (f.head(3).norm() > 3.0)
    v = this->forceFeedbackControl(frame, pose, f);
  else
    v = this->PIControl(frame, pose, twist);

  tf::vectorEigenToKDL(v, twist.vel);
}

VectorXd FeedbackController::PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
  double kp = 3.0, ki = 0.0;
  Vector3d p_cur, p_des, v_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);
  tf::vectorKDLToEigen(twist.vel, v_des);

  static Vector3d e_integ = Vector3d::Zero();
  e_integ += p_des - p_cur;

  return v_des + kp * (p_des - p_cur) + ki * e_integ;
}

VectorXd FeedbackController::forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f) {
  double kf = 0.1, alpha = 1000.0;
  Vector3d p_cur, p_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);

  return kf * (alpha * (p_des - p_cur) + f.head(3));
}