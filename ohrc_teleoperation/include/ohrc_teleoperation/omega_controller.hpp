#ifndef OMEGA_CONTROLLER_HPP
#define OMEGA_CONTROLLER_HPP

#include "omega_haptic_device/Omega.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "ohrc_control/cart_controller.hpp"

class OmegaController : public virtual CartController {
private:
  ros::Subscriber subOmega, subForce, subLeaderEnergy;
  ros::Publisher pubOmegaPose, pubOmegaForce, pubEnergy;

  void init();
  void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) override;

  void cbOmegaState(const omega_haptic_device::Omega::ConstPtr& msg);
  void cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void cbEnergy(const std_msgs::Float32::ConstPtr& msg);

  void initWithJnt(const KDL::JntArray& q_init) override;

  geometry_msgs::WrenchStamped _force, _force_omega, _force_assist;
  bool flagForce = false;

  double _leaderEnergy = 0.0;

  Affine3d Tft_eff, Tomega_base;

  enum HapticType { PositionPositionFeedback, PositionForce, PositionForceFeedback, None } hapticType;

public:
  OmegaController();
  // OmegaController(std::string robot);
};

#endif  // OMEGA_CONTROLLER_HPP