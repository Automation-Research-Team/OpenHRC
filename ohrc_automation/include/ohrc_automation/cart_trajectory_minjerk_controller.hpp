#ifndef CART_TRAJECTORY_MINJERK_CONTROLLER_HPP
#define CART_TRAJECTORY_MINJERK_CONTROLLER_HPP

#include "ohrc_automation/cart_trajectory_controller.hpp"

class CartTrajectoryMinjerkController : virtual public CartTrajectoryController {
protected:
  moveit_msgs::CartesianTrajectory interpolateTrajectory(const moveit_msgs::CartesianTrajectory& trj) override;

public:
  using CartTrajectoryController::CartTrajectoryController;
};

#endif  // CART_TRAJECTORY_MINJERK_CONTROLLER_HPP
