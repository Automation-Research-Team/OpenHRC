
#include "ohrc_teleoperation/k4abt_body_interface.hpp"

bool K4abtBodyInterface::getEnableFlag(const ohrc_msgs::BodyPartState& handState, const ohrc_msgs::BodyPartState& anotherBodyPartState) {
  if ((anotherBodyPartState.pose.position.z > 0.0) && (anotherBodyPartState.pose.position.z - handState.pose.position.z > 0.))
    return true;
  else
    return false;
}
