#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include "ohrc_control/cart_controller.hpp"

class Interface {
protected:
  ros::NodeHandle n;

  std::shared_ptr<CartController> controller;

public:
  Interface(std::shared_ptr<CartController> controller) : n("~") {
    this->controller = controller;
  };

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist){};
  virtual void initInterface(){};
  virtual void resetInterface(){};

  int curTargetId = 0, nCompletedTask = 0;
  bool blocked = false;
  std::string targetName;
  double targetDistance = 0.0;
};

#endif  // INTERFACE_HPP