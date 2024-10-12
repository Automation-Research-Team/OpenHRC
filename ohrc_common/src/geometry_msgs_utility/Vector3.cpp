/**
 * @file Vector3.cpp
 * @author Shunki Itadera
 * @date Oct. 2018
 * @brief utility library for Vector3 message
 **/

#include "ohrc_common/geometry_msgs_utility/Vector3.h"

namespace geometry_msgs_utility {
Vector3::Vector3(int order, double cutoff_freq, double sampling_freq) {
  for (int i = 0; i < 3; ++i)
    _filter.push_back(std::unique_ptr<butterworth>(new butterworth(order, cutoff_freq, sampling_freq)));

  delta_t = 1.0 / sampling_freq;
}

Vector3::~Vector3() {
  // deleted because this vector conteners std::unique_ptr_filter_vec
  //  for (int i = 0; i < 3; ++i)
  //    delete _filter[i];
}

void Vector3::LPF(geometry_msgs::msg::Vector3 raw_point, geometry_msgs::msg::Vector3 &filtered_point) {
  filtered_point.x = _filter[0]->filter(raw_point.x);
  filtered_point.y = _filter[1]->filter(raw_point.y);
  filtered_point.z = _filter[2]->filter(raw_point.z);
}

void Vector3::diff(geometry_msgs::msg::Vector3 point, geometry_msgs::msg::Vector3 &diff_point) {
  static geometry_msgs::msg::Vector3 old_point = point;
  diff_point.x = (point.x - old_point.x) / delta_t;
  diff_point.y = (point.y - old_point.y) / delta_t;
  diff_point.z = (point.z - old_point.z) / delta_t;
  old_point = point;
}

void Vector3::diff_LPF(geometry_msgs::msg::Vector3 point, geometry_msgs::msg::Vector3 &filtered_point) {
  geometry_msgs::msg::Vector3 diff_point;
  diff(point, diff_point);
  LPF(diff_point, filtered_point);
}

double Vector3::dist(geometry_msgs::msg::Vector3 point1, geometry_msgs::msg::Vector3 point2) {
  return sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0) + pow(point1.z - point2.z, 2.0));
}

}  // namespace geometry_msgs_utility
