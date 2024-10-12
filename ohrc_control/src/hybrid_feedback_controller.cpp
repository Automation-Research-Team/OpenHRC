#include "ohrc_control/hybrid_feedback_controller.hpp"

void HybridFeedbackController::initInterface() {
  t0_f.resize(6, -1000.0);
}

void HybridFeedbackController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd f = controller->getForceEefVec();
  VectorXd v(6);

  VectorXd e = MyIK::getCartError(frame, pose);
  VectorXd v_force = this->forceFeedbackControl(e, f);
  VectorXd v_pi = this->PIControl(e, twist);
  VectorXd v_ada_pi = this->adaptivePIControl(e, twist);

  if (controller->getOperationEnable()) {
    for (int i = 0; i < 6; i++) {
      if (std::abs(f[i]) < 1.0)
        v[i] = v_ada_pi[i];
      else {
        v[i] = v_force[i];
        t0_f[i] = rclcpp::Time::now().toSec();
      }
    }
  }

  tf::twistEigenToKDL(v, twist);
}

// https://qiita.com/Kurounmo/items/70ca355f5be97df1e9da
double pseudoSigmoid(double x, double x0, double w, double y0, double h) {
  double y;
  x = (x - x0) / w;

  double eps = 1.0e-5;
  if (x < eps)
    y = y0;
  else if (x > 1.0 - eps)
    y = h + y0;
  else {
    double f = std::exp(-1.0 / x);
    double g = std::exp(-1.0 / (1.0 - x));
    y = h * f / (f + g) + y0;
  }
  return y;
}

VectorXd HybridFeedbackController::adaptivePIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
  const double kp = 1.2, ki = 0.0;  // 1.0*dt;
  Vector3d p_cur, p_des, v_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);
  tf::vectorKDLToEigen(twist.vel, v_des);

  std::vector<double> gain(3);

  const double a = 0.2, b = 2.0;
  for (int i = 0; i < 3; i++) {
    double y0 = a * std::exp(-b * (rclcpp::Time::now().toSec() - t0_f[i]));  // remain feedback just after losing physical contact
    gain[i] = pseudoSigmoid(std::abs(v_des[i]), 0.0, 0.4, y0, 2.0);
  }

  return v_des + Vector3d(gain.data()).asDiagonal() * kp * (p_des - p_cur);  // + ki * e_integ;
}

VectorXd HybridFeedbackController::adaptivePIControl(const VectorXd& e, const KDL::Twist& twist) {
  const double kp = 1.2, ki = 0.0;  // 1.0*dt;

  Matrix<double, 6, 1> v_des;
  tf::twistKDLToEigen(twist, v_des);

  VectorXd gain(6);

  const double a = 0.2, b = 3.0;
  for (int i = 0; i < 6; i++) {
    double y0 = a * std::exp(-b * (rclcpp::Time::now().toSec() - t0_f[i]));  // remain feedback just after losing physical contact
    gain[i] = pseudoSigmoid(std::abs(v_des[i]), 0.0, 0.4, y0, 2.0);
  }

  gain.tail(3) = gain.tail(3) * 0.5 / M_PI;
  return v_des + gain.asDiagonal() * kp * e;  // + ki * e_integ;
}

VectorXd HybridFeedbackController::PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
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

VectorXd HybridFeedbackController::PIControl(const VectorXd& e, const KDL::Twist& twist) {
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

VectorXd HybridFeedbackController::forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f) {
  double kf = 0.001, alpha = 100.0;  // estimated force command corresponding to 1 m position error.
  Vector3d p_cur, p_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);

  return kf * (alpha * (p_des - p_cur) + f.head(3));
}

VectorXd HybridFeedbackController::forceFeedbackControl(const VectorXd& e, const VectorXd f) {
  double kf = 0.001, alpha = 100.0;  // estimated force command corresponding to 1 m position and pi rad angle error.

  VectorXd gain = VectorXd::Ones(6) * kf;
  gain.tail(3) = gain.tail(3) / M_PI;

  return gain.asDiagonal() * (alpha * e + f.head(3));
}