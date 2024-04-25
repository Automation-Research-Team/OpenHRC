#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include "ohrc_control/cart_controller.hpp"

class Interface {
protected:
  ros::NodeHandle n;
  const double dt;

  std::shared_ptr<CartController> controller;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  TaskState taskState = TaskState::Initial;

  std::string targetName;
  double targetDistance = 0.0;

  inline void reset() {
    controller->resetPose();
    controller->resetFt();
    resetInterface();
  }

public:
  Interface(std::shared_ptr<CartController> controller) : n("~"), dt(controller->dt) {
    this->controller = controller;
  }

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist){};
  virtual void initInterface(){};
  virtual void resetInterface(){};
  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist){};

  int targetIdx = -1, nCompletedTask = 0;
  bool blocked = false;

  inline std::string getTargetName() {
    return this->targetName;
  }

  inline double getTargetDistance() {
    return this->targetDistance;
  }

  inline TaskState getTaskState() {
    return this->taskState;
  }
};

#endif  // INTERFACE_HPP