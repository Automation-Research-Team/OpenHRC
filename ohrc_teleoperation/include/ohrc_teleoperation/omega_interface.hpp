#ifndef OMEGA_INTERFACE2_HPP
#define OMEGA_INTERFACE2_HPP

#include "ohrc_teleoperation/state_topic_interface.hpp"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

class OmegaInterface : public StateTopicInterface {
protected:
  // void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des);
  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) override;
  ros::Publisher pubOmegaPose, pubOmegaForce, pubEnergy, pubOmegaForceVis;

  enum HapticType { PositionPositionFeedback, PositionForce, PositionForceFeedback, None } hapticType;

  virtual void modifyTargetState(ohrc_msgs::State& state) override;

  VectorXd ft_omega;

public:
  using StateTopicInterface::StateTopicInterface;
  void initInterface() override;
};

#endif  // OMEGA_INTERFACE_HPP