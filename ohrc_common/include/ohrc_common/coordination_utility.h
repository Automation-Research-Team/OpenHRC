#ifndef COORDINATION_UTILITY_H
#define COORDINATION_UTILITY_H

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

namespace rotation_util {

////////////////////////////////////////////////
/// \brief checkQuaternionSign
/// \param q
/// \return
///
bool checkQuaternionSign(Eigen::Quaterniond q);

////////////////////////////////////////////////
/// \brief flipQuaterionSign
/// \param q
/// \return
///
Eigen::Quaterniond flipQuaterionSign(Eigen::Quaterniond q);

////////////////////////////////////////////////
/// \brief checkFlipQuaternionSign
/// \param q
/// \return
///
Eigen::Quaterniond checkFlipQuaternionSign(Eigen::Quaterniond q);

////////////////////////////////////////////////
/// \brief getQuaternionError
/// \param desired_q
/// \param current_q
/// \return
///
Eigen::Vector3d getQuaternionError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q);

Eigen::Vector3d getQuaternionLogError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q);

void getQuaternionError(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q, Eigen::Quaterniond &error_q);

double getQuaternionErrorYaw(const double &yaw1, const double &yaw2);

Eigen::Vector3d getErrorEuler(const Eigen::Matrix3d &desired_R, const Eigen::Matrix3d &current_R);
Eigen::Vector3d getErrorEuler(const Eigen::Quaterniond &desired_q, const Eigen::Quaterniond &current_q);

Eigen::Vector3d getRotationMatrixError(const Eigen::Matrix3d &desired_R, const Eigen::Matrix3d &current_R);

};  // namespace rotation_util

#endif  // COORDINATION_UTILITY_H
