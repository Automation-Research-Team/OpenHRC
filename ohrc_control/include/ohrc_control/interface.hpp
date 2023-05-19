#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include "ohrc_control/cart_controller.hpp"

class Interface {
protected:
  ros::NodeHandle n;
  double dt;

  std::shared_ptr<CartController> controller;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  TaskState taskState = TaskState::Initial;

  std::string targetName;
  double targetDistance = 0.0;

public:
  Interface(std::shared_ptr<CartController> controller) : n("~") {
    this->controller = controller;
    dt = controller->dt;
  };

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist){};
  virtual void initInterface(){};
  virtual void resetInterface(){};
  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist){};

  int curTargetId = 0, nCompletedTask = 0;
  bool blocked = false;

  std::string getTargetName() {
    return this->targetName;
  }

  double getTargetDistance() {
    return this->targetDistance;
  }

  TaskState getTaskState() {
    return this->taskState;
  }
};

#endif  // INTERFACE_HPP