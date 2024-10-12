#ifndef VECTOR3_H
#define VECTOR3_H

#include <geometry_msgs/msg/vector3.hpp>
//#include "ohrc_common/filter_utility.h"
#include "ohrc_common/filter_utility/butterworth.h"

namespace geometry_msgs_utility {
class Vector3 {
private:
  double delta_t;
  std::vector<std::unique_ptr<butterworth>> _filter;

public:
  void LPF(geometry_msgs::msg::Vector3 raw_point, geometry_msgs::msg::Vector3 &filtered_point);
  void diff(geometry_msgs::msg::Vector3 point, geometry_msgs::msg::Vector3 &diff_point);
  void diff_LPF(geometry_msgs::msg::Vector3 point, geometry_msgs::msg::Vector3 &filtered_point);
  double dist(geometry_msgs::msg::Vector3 point1, geometry_msgs::msg::Vector3 point2);
  Vector3(int order, double cutoff_freq, double sampling_freq);
  ~Vector3();
};
}  // namespace geometry_msgs_utility

#endif  // VECTOR3_H
