#ifndef CART_TRAJECTORY_CONTROLLER_HPP
#define CART_TRAJECTORY_CONTROLLER_HPP

#include <moveit_msgs/CartesianTrajectory.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/interface.hpp"

class CartTrajectoryController : virtual public Interface {
  Affine3d T_init ;
  int i = 0;
  bool start = true;
protected:
  std::mutex mtx_cart;
  ros::Subscriber trjSubscriber;

  moveit_msgs::CartesianTrajectory _trj;

  void cbCartTrajectory(const moveit_msgs::CartesianTrajectory::ConstPtr& msg);

  virtual void setSubscriber();

public:
  using Interface::Interface;
  // CartTrajectoryController(std::shared_ptr<CartController> controller) : Interface(controller) {
  // }
  virtual void initInterface() override;
  virtual void resetInterface() override;

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
};

#endif  // CART_TRAJECTORY_CONTROLLER_HPP
