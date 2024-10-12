/**
 * @file WrenchStamped.cpp
 * @author Shunki Itadera
 * @date Oct. 2018
 * @brief utility library for WrenchStamped message
 **/

#include "ohrc_common/geometry_msgs_utility/WrenchStamped.h"

namespace geometry_msgs_utility {

void deadZone(geometry_msgs::msg::WrenchStamped wrenchStamped, geometry_msgs::msg::WrenchStamped &filtered_wrenchStamped, paramDeadZone param) {
  filtered_wrenchStamped.header = wrenchStamped.header;
  deadZone(wrenchStamped.wrench, filtered_wrenchStamped.wrench, param);

  return;
}

void WrenchStamped::deadZone_LPF(geometry_msgs::msg::WrenchStamped wrenchStamped, geometry_msgs::msg::WrenchStamped &filtered_wrenchStamped) {
  filtered_wrenchStamped.header = wrenchStamped.header;
  Wrench::deadZone_LPF(wrenchStamped.wrench, filtered_wrenchStamped.wrench);

  return;
}

void WrenchStamped::LPF(geometry_msgs::msg::WrenchStamped wrenchStamped, geometry_msgs::msg::WrenchStamped &filtered_wrenchStamped) {
  filtered_wrenchStamped.header = wrenchStamped.header;
  Wrench::LPF(wrenchStamped.wrench, filtered_wrenchStamped.wrench);

  return;
}

}  // namespace geometry_msgs_utility
