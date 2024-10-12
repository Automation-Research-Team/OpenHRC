#ifndef WRENCHSTAMPED_UTILITY_H
#define WRENCHSTAMPED_UTILITY_H

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "ohrc_common/geometry_msgs_utility/Wrench.h"

namespace geometry_msgs_utility {

// using geometry_msgs_utility::deadZone;
void deadZone(geometry_msgs::msg::WrenchStamped wrenchStamped, geometry_msgs::msg::WrenchStamped &filtered_wrenchStamped, paramDeadZone param);

class WrenchStamped : public Wrench {
public:
  WrenchStamped(WrenchStamped::paramLPF param_LPF, geometry_msgs_utility::paramDeadZone param_deadZone) : Wrench(param_LPF, param_deadZone) {
  }
  WrenchStamped(WrenchStamped::paramLPF param_LPF) : Wrench(param_LPF) {
  }

  using Wrench::deadZone_LPF;
  void deadZone_LPF(geometry_msgs::msg::WrenchStamped raw_wrench, geometry_msgs::msg::WrenchStamped &filtered_wrench);
  void LPF(geometry_msgs::msg::WrenchStamped raw_wrench, geometry_msgs::msg::WrenchStamped &filtered_wrench);
};
}  // namespace geometry_msgs_utility

#endif  // WRENCHSTAMPED_UTILITY_H
