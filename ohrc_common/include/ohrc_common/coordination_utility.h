#ifndef COORDINATION_UTILITY_H
#define COORDINATION_UTILITY_H

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

namespace rotation_util {

////////////////////////////////////////////////
/// \brief checkQuaternionSign
/// \param q
/// \return
///
inline bool checkQuaternionSign(Eigen::Quaterniond q) {
  if (q.w() > 0.0) {
    return true;
  }
  return false;
}

////////////////////////////////////////////////
/// \brief flipQuaterionSign
/// \param q
/// \return
///
inline Eigen::Quaterniond flipQuaterionSign(Eigen::Quaterniond q) {
  return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
}

////////////////////////////////////////////////
/// \brief checkFlipQuaternionSign
/// \param q
/// \return
///
inline Eigen::Quaterniond checkFlipQuaternionSign(Eigen::Quaterniond q) {
  if (!checkQuaternionSign(q)) {
    return flipQuaterionSign(q);
  }
  return q;
}

inline void getQuaternionError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q, Eigen::Quaterniond &error_q) {
  Eigen::Quaterniond delta_q = desired_q * current_q.inverse();
  delta_q.normalize();

  delta_q = checkFlipQuaternionSign(delta_q);

  error_q = delta_q;

  return;
}

inline Eigen::Vector3d getQuaternionLogError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q) {
  Eigen::Quaterniond delta_q;
  getQuaternionError(desired_q, current_q, delta_q);

  double theta = 2.0 * atan2(delta_q.vec().norm(), delta_q.w());

  Eigen::Vector3d eo;

  if (abs(1.0 - delta_q.w() * delta_q.w()) < 0.001)
    eo = 2.0 * delta_q.vec();
  else
    eo = theta / sin(theta / 2.0) * delta_q.vec();
  return eo;
}

////////////////////////////////////////////////
/// \brief getQuaternionError
/// \param desired_q
/// \param current_q
/// \return
///
inline Eigen::Vector3d getQuaternionError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q) {
  Eigen::Quaterniond delta_q;
  getQuaternionError(desired_q, current_q, delta_q);

  Eigen::Vector3d eo = delta_q.vec();

  return eo;
}

inline double getQuaternionErrorYaw(const double &yaw1, const double &yaw2) {
  Eigen::Quaterniond q1 = Eigen::Quaterniond(Eigen::AngleAxisd(yaw1, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q2 = Eigen::Quaterniond(Eigen::AngleAxisd(yaw2, Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond q1m2;  // q1 - q2
  getQuaternionError(q1, q2, q1m2);

  double yaw1m2 = tf2::getYaw(tf2::toMsg(q1m2));

  return yaw1m2;
}

inline Eigen::Vector3d getErrorEuler(const Eigen::Matrix3d &desired_R, const Eigen::Matrix3d &current_R) {
  return (desired_R * current_R.transpose()).eulerAngles(0, 1, 2);
}

inline Eigen::Vector3d getErrorEuler(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q) {
  Eigen::Quaterniond delta_q;
  getQuaternionError(desired_q, current_q, delta_q);

  Eigen::Vector3d eo = delta_q.toRotationMatrix().eulerAngles(0, 1, 2);

  return eo;
}

inline Eigen::Vector3d getRotationMatrixError(const Eigen::Matrix3d &desired_R, const Eigen::Matrix3d &current_R) {
  Eigen::Matrix3d eR = desired_R * current_R.transpose();

  Eigen::Vector3d l(eR(2, 1) - eR(1, 2), eR(0, 2) - eR(2, 0), eR(1, 0) - eR(0, 1));

  Eigen::Vector3d eo = Eigen::Vector3d::Zero();
  if (l.norm() > 1.0e-8)
    eo = atan2(l.norm(), eR.trace() - 1.0) * l.normalized();
  else if (eR.trace() == -1.0) {
    if (eR(1, 0) >= 0.0 && eR(2, 1) >= 0.0 && eR(0, 2) >= 0.0)
      eo = M_PI * Eigen::Vector3d(sqrt((eR(0, 0) + 1.0) * 0.5), sqrt((eR(1, 1) + 1.0) * 0.5), sqrt((eR(2, 2) + 1.0) * 0.5));
    else if (eR(1, 0) >= 0.0 && eR(2, 1) < 0.0 && eR(0, 2) <= 0.0)
      eo = M_PI * Eigen::Vector3d(sqrt((eR(0, 0) + 1.0) * 0.5), sqrt((eR(1, 1) + 1.0) * 0.5), -sqrt((eR(2, 2) + 1.0) * 0.5));
    else if (eR(1, 0) < 0.0 && eR(2, 1) <= 0.0 && eR(0, 2) >= 0.0)
      eo = M_PI * Eigen::Vector3d(sqrt((eR(0, 0) + 1.0) * 0.5), -sqrt((eR(1, 1) + 1.0) * 0.5), sqrt((eR(2, 2) + 1.0) * 0.5));
    else if (eR(1, 0) <= 0.0 && eR(2, 1) >= 0.0 && eR(0, 2) < 0.0)
      eo = M_PI * Eigen::Vector3d(sqrt((-eR(0, 0) + 1.0) * 0.5), sqrt((eR(1, 1) + 1.0) * 0.5), sqrt((eR(2, 2) + 1.0) * 0.5));
  }

  return eo;
}

};  // namespace rotation_util

#endif  // COORDINATION_UTILITY_H
