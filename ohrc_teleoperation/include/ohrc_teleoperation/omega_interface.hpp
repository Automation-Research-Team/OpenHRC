#ifndef MULTI_OMEGA_CONTROLLER_HPP
#define MULTI_OMEGA_CONTROLLER_HPP

#include "ohrc_control/multi_cart_controller.hpp"
#include "ohrc_msgs/State.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

class OmegaInterface : public virtual MultiCartController {
  void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  // void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, CartController* controller) override;

  ros::Subscriber subOmega, subForce, subLeaderEnergy;
  ros::Publisher pubOmegaPose, pubOmegaForce, pubEnergy, pubOmegaForceVis;
  // Affine3d Tft_eff, Tomega_base;
  std::vector<Affine3d> T_omega_base;

  TransformUtility trans;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  enum HapticType { PositionPositionFeedback, PositionForce, PositionForceFeedback, None } hapticType;

  struct s_updateManualTargetPose {
    Affine3d T, T_start, T_omega_start;
    bool isFirst = true;
  };
  std::vector<s_updateManualTargetPose> s_updateManualTargetPoses;

  std::mutex mtx_omega;
  ohrc_msgs::State _omegaState;

  void cbOmegaState(const ohrc_msgs::State::ConstPtr& msg);

  void starting() override;

public:
  OmegaInterface();
};

#endif  // MULTI_OMEGA_CONTROLLER_HPP