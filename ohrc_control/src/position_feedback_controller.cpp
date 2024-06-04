#include "ohrc_control/position_feedback_controller.hpp"

void PositionFeedbackController::initInterface() {
  t0_f.resize(6, -1000.0);
}

void PositionFeedbackController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd f = controller->getForceEefVec();
  VectorXd v(6);

  VectorXd e = MyIK::getCartError(frame, pose);
  VectorXd v_pi = this->PIControl(e, twist);

  if (controller->getOperationEnable()) {
    v = v_pi;
  }

  tf::twistEigenToKDL(v, twist);
}

VectorXd PositionFeedbackController::PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
  const double kp = 1.2, ki = 0.0;  // 1.0*dt;
  Vector3d p_cur, p_des, v_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);
  tf::vectorKDLToEigen(twist.vel, v_des);

  std::vector<double> gain(3);

  const double a = 0.2, b = 2.0;
  for (int i = 0; i < 3; i++) {
    gain[i] = a;
  }

  return v_des + Vector3d(gain.data()).asDiagonal() * kp * (p_des - p_cur);  // + ki * e_integ;
}

VectorXd PositionFeedbackController::PIControl(const VectorXd& e, const KDL::Twist& twist) {
  const double kp = 2.0;

  Matrix<double, 6, 1> v_des;
  tf::twistKDLToEigen(twist, v_des);

  VectorXd gain(6);

  for (int i = 0; i < 6; i++) {
    gain[i] = kp;
  }

  gain.tail(3) = gain.tail(3) * 0.5 / M_PI;
  return v_des + gain.asDiagonal() * kp * e;  // + ki * e_integ;
}
