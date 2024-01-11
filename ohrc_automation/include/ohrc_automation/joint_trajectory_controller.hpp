#ifndef JOINT_TRAJECTORY_CONTROLLER_HPP
#define JOINT_TRAJECTORY_CONTROLLER_HPP

#include <moveit_msgs/CartesianTrajectory.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/interface.hpp"

class JointTrajectoryController : public virtual Interface {
protected:
  std::mutex mtx_joint;
  ros::Subscriber trjSubscriber;

  trajectory_msgs::JointTrajectory _jointTrj;
  moveit_msgs::CartesianTrajectory _cartTrj;

  void cbJointTrajectory(const moveit_msgs::MoveGroupActionResult::ConstPtr& msg);

  virtual void setSubscriber();
  bool _start = true;

public:
  using Interface::Interface;
  // CartTrajectoryController(std::shared_ptr<CartController> controller) : Interface(controller) {
  // }
  virtual void initInterface() override;

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
};

#endif  // JOINT_TRAJECTORY_CONTROLLER_HPP
