#ifndef CART_TRAJECTORY_IMPEDANCE_CONTROLLER_HPP
#define CART_TRAJECTORY_IMPEDANCE_CONTROLLER_HPP

#include "ohrc_automation/cart_trajectory_controller.hpp"
#include "ohrc_automation/impedance_controller.hpp"

class CartTrajectoryImpedanceController : public CartTrajectoryController, public ImpedanceController {
  bool updateImpedanceTarget(const VectorXd& x, VectorXd& xd) override;

  void setSubscriber() override final {
    CartTrajectoryController::setSubscriber();
  };

public:
  using CartTrajectoryController::CartTrajectoryController;
  using ImpedanceController::ImpedanceController;

  void initInterface() override final {
    ImpedanceController::initInterface();
  };

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override final {
    ImpedanceController::updateTargetPose(pose, twist);
  };
};

#endif  // CART_TRAJECTORY_IMPEDANCE_CONTROLLER_HPP
