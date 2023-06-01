#ifndef CART_TRAJECTORY_CONTROLLER_HPP
#define CART_TRAJECTORY_CONTROLLER_HPP

#include <moveit_msgs/CartesianTrajectory.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/interface.hpp"

class CartTrajectoryController : public Interface {
  std::mutex mtx_cart;
  ros::Subscriber trjSubscriber;

  moveit_msgs::CartesianTrajectory _trj;

  void cbCartTrajectory(const moveit_msgs::CartesianTrajectory::ConstPtr& msg);

protected:
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;

public:
  using Interface::Interface;
  void initInterface() override;
};

#endif  // CART_TRAJECTORY_CONTROLLER_HPP
