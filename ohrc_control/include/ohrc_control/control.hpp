#ifndef CONTROL_H
#define CONTROL_H

#include "ohrc_control/interface.hpp"
#include "ohrc_control/multi_cart_controller.hpp"

class Control : virtual public MultiCartController {
protected:
  std::vector<std::shared_ptr<Interface>> interfaces;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->updateTargetPose(pose, twist);
  }

  void initInterface(std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->initInterface();
  }

  void resetInterface(std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->resetInterface();
  }
};

#endif  // CONTROL_H