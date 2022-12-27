#ifndef WRENCH_UTILITY_H
#define WRENCH_UTILITY_H

#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>

#include "ohrc_common/geometry_msgs_utility/Vector3.h"
#include "ohrc_common/math_utility.h"
#include "ohrc_common/transform_utility.h"

namespace tf2 {

inline Eigen::VectorXd fromMsg(geometry_msgs::Wrench raw_wrench, Eigen::VectorXd &vector) {
  vector(0) = raw_wrench.force.x;
  vector(1) = raw_wrench.force.y;
  vector(2) = raw_wrench.force.z;
  vector(3) = raw_wrench.torque.x;
  vector(4) = raw_wrench.torque.y;
  vector(5) = raw_wrench.torque.z;

  return vector;
}

inline Eigen::VectorXd fromMsg(geometry_msgs::Wrench raw_wrench) {
  Eigen::VectorXd vector(6);
  return fromMsg(raw_wrench, vector);
}

inline geometry_msgs::Wrench toMsg(Eigen::VectorXd vector, geometry_msgs::Wrench &wrench) {
  wrench.force.x = vector(0);
  wrench.force.y = vector(1);
  wrench.force.z = vector(2);
  wrench.torque.x = vector(3);
  wrench.torque.y = vector(4);
  wrench.torque.z = vector(5);

  return wrench;
}
}  // namespace tf2

namespace geometry_msgs_utility {

struct paramDeadZone {
  double force_upper;
  double force_lower;
  double torque_upper;
  double torque_lower;
};

void deadZone(geometry_msgs::Wrench raw_wrench, geometry_msgs::Wrench &filtered_wrench, paramDeadZone param);

inline geometry_msgs::Wrench transformFT(geometry_msgs::Wrench ft_in, const Affine3d trans) {
  geometry_msgs::Wrench ft_out;
  tf2::toMsg(TransformUtility::transformFT(tf2::fromMsg(ft_in), trans), ft_out);

  return ft_out;
}

class Wrench {
public:
  struct paramLPF {
    int order;
    double cutoff_freq;
    double sampling_freq;
  };
  paramDeadZone m_paramDeadZone;

private:
  double delta_t;
  std::vector<std::unique_ptr<geometry_msgs_utility::Vector3>> _filter_vec;
  void LPF_initialize(int order, double cutoff_freq, double sampling_freq);

public:
  Wrench(paramLPF param_LPF);
  Wrench(paramLPF param_LPF, paramDeadZone param_DeadZone);
  ~Wrench();
  void LPF(geometry_msgs::Wrench raw_wrench, geometry_msgs::Wrench &filtered_wrench);
  void diff(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &diff_wrench);
  void diff_LPF(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &filtered_wrench);
  void deadZone_LPF(geometry_msgs::Wrench raw_wrench, geometry_msgs::Wrench &filtered_wrench);
};
}  // namespace geometry_msgs_utility

#endif  // WRENCH_UTILITY_H
