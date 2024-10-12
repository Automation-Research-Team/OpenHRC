/**
 * @file Wrench.cpp
 * @author Shunki Itadera
 * @date Oct. 2018
 * @brief utility library for Wrench message
 **/

#include "ohrc_common/geometry_msgs_utility/Wrench.h"

namespace geometry_msgs_utility {
Wrench::Wrench(paramLPF param_LPF) {
  LPF_initialize(param_LPF.order, param_LPF.cutoff_freq, param_LPF.sampling_freq);
  delta_t = 1.0 / param_LPF.sampling_freq;
}

Wrench::Wrench(paramLPF param_LPF, paramDeadZone param_DeadZone) {
  LPF_initialize(param_LPF.order, param_LPF.cutoff_freq, param_LPF.sampling_freq);
  delta_t = 1.0 / param_LPF.sampling_freq;

  m_paramDeadZone = param_DeadZone;
}

Wrench::~Wrench() {
  // deleted because this vector conteners std::unique_ptr_filter_vec
  //  for (int i = 0; i < _filter_vec.size(); ++i)
  //    delete _filter_vec[i];
}

void Wrench::LPF_initialize(int order, double cutoff_freq, double sampling_freq) {
  for (int i = 0; i < 2; ++i)
    _filter_vec.push_back(std::unique_ptr<geometry_msgs_utility::Vector3>(new geometry_msgs_utility::Vector3(order, cutoff_freq, sampling_freq)));
  // for force & torque
}

void Wrench::LPF(geometry_msgs::msg::Wrench raw_wrench, geometry_msgs::msg::Wrench &filtered_wrench) {
  _filter_vec[0]->LPF(raw_wrench.force, filtered_wrench.force);
  _filter_vec[1]->LPF(raw_wrench.torque, filtered_wrench.torque);
}

void Wrench::diff(geometry_msgs::msg::Wrench wrench, geometry_msgs::msg::Wrench &diff_wrench) {
  static geometry_msgs::msg::Wrench old_wrench = wrench;

  diff_wrench.force.x = (wrench.force.x - old_wrench.force.x) / delta_t;
  diff_wrench.force.y = (wrench.force.y - old_wrench.force.y) / delta_t;
  diff_wrench.force.z = (wrench.force.z - old_wrench.force.z) / delta_t;
  diff_wrench.torque.x = (wrench.torque.x - old_wrench.torque.x) / delta_t;
  diff_wrench.torque.y = (wrench.torque.y - old_wrench.torque.y) / delta_t;
  diff_wrench.torque.z = (wrench.torque.z - old_wrench.torque.z) / delta_t;

  old_wrench = wrench;
}

void Wrench::diff_LPF(geometry_msgs::msg::Wrench wrench, geometry_msgs::msg::Wrench &filtered_wrench) {
  geometry_msgs::msg::Wrench diff_wrench;
  diff(wrench, diff_wrench);
  LPF(diff_wrench, filtered_wrench);
}

void deadZone(geometry_msgs::msg::Wrench raw_wrench, geometry_msgs::msg::Wrench &filtered_wrench, paramDeadZone param) {
  filtered_wrench.force.x = math_utility::deadZone(raw_wrench.force.x, param.force_upper, param.force_lower);
  filtered_wrench.force.y = math_utility::deadZone(raw_wrench.force.y, param.force_upper, param.force_lower);
  filtered_wrench.force.z = math_utility::deadZone(raw_wrench.force.z, param.force_upper, param.force_lower);
  filtered_wrench.torque.x = math_utility::deadZone(raw_wrench.torque.x, param.torque_upper, param.torque_lower);
  filtered_wrench.torque.y = math_utility::deadZone(raw_wrench.torque.y, param.torque_upper, param.torque_lower);
  filtered_wrench.torque.z = math_utility::deadZone(raw_wrench.torque.z, param.torque_upper, param.torque_lower);

  return;
}

void Wrench::deadZone_LPF(geometry_msgs::msg::Wrench raw_wrench, geometry_msgs::msg::Wrench &filtered_wrench) {
  geometry_msgs::msg::Wrench dead_wrench;
  deadZone(raw_wrench, dead_wrench, m_paramDeadZone);
  LPF(dead_wrench, filtered_wrench);
}

}  // namespace geometry_msgs_utility
