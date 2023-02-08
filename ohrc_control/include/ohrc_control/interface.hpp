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
};

#endif  // INTERFACE_HPP