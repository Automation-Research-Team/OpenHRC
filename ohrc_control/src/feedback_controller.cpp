#include "ohrc_control/feedback_controller.hpp"

void FeedbackController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd f = controller->getForceEefVec();
  VectorXd v;

  if (f.head(3).norm() > 2.0)
    v = this->forceFeedbackControl(frame, pose, f);
  else
    v = this->PIControl(frame, pose, twist);

  tf::vectorEigenToKDL(v, twist.vel);
}

VectorXd FeedbackController::PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
  double kp = 1.25, ki = 0.0;//1.0*dt;
  Vector3d p_cur, p_des, v_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);
  tf::vectorKDLToEigen(twist.vel, v_des);

  double d_gain_x = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[0]) / 1.0));
  double d_gain_y = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[1]) / 1.0));
  double d_gain_z = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[2]) / 1.0));

  MatrixXd D_inv = Vector3d(d_gain_x,d_gain_y,d_gain_z).cwiseInverse().asDiagonal();

  static Vector3d e_integ = Vector3d::Zero();
  e_integ += p_des - p_cur;

  for (int i=0;i<3;i++){
  if(std::abs(v_des[i]) < 0.01){
    e_integ[i]=0.0;
    kp=0.0;
  }
  }

  return v_des + D_inv*kp * (p_des - p_cur) + ki * e_integ;
}

VectorXd FeedbackController::forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f) {
  double kf = 0.001, alpha = 100.0;
  Vector3d p_cur, p_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);

  return kf * (alpha * (p_des - p_cur) + f.head(3));
}