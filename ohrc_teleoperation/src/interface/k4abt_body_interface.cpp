
#include "ohrc_teleoperation/k4abt_body_interface.hpp"

bool K4abtBodyInterface::getEnableFlag(const ohrc_msgs::HandState& handState, const ohrc_msgs::HandState& anotherHandState) {
  if ((anotherHandState.pose.position.z > 0.0) && (anotherHandState.pose.position.z - handState.pose.position.z > 0.))
    return true;
  else
    return false;
}
