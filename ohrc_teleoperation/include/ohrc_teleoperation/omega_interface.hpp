#ifndef OMEGA_INTERFACE2_HPP
#define OMEGA_INTERFACE2_HPP

#include "ohrc_teleoperation/state_topic_interface.hpp"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

class OmegaInterface : public virtual StateTopicInterface {
  void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller) override;

  ros::Publisher pubOmegaPose, pubOmegaForce, pubEnergy, pubOmegaForceVis;

  enum HapticType { PositionPositionFeedback, PositionForce, PositionForceFeedback, None } hapticType;

  void modifyTargetState(ohrc_msgs::State& state) override;

public:
  OmegaInterface();
};

#endif  // OMEGA_INTERFACE_HPP