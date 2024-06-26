#ifndef K4ABT_BODY_INTERFACE_HPP
#define K4ABT_BODY_INTERFACE_HPP

#include "ohrc_teleoperation/xr_body_interface.hpp"

class K4abtBodyInterface : virtual public XrBodyInterface {
  bool getEnableFlag(const ohrc_msgs::BodyPartState& handState, const ohrc_msgs::BodyPartState& anotherBodyPartState) override;

public:
  using XrBodyInterface::XrBodyInterface;
};

#endif  // K4ABT_BODY_INTERFACE_HPP