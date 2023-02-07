#ifndef CONTROL_H
#define CONTROL_H

#include "ohrc_control/multi_cart_controller.hpp"

template <class T>
class Control : virtual public MultiCartController {
protected:
  std::vector<std::shared_ptr<T>> interfaces;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->updateTargetPose(pose, twist);
  }

  void initInterface(std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->initInterface();
  }

  void resetInterface(std::shared_ptr<CartController> controller) override {
    interfaces[controller->getIndex()]->resetInterface();
  }

public:
  Control() {
    interfaces.resize(nRobot);
    for (int i = 0; i < nRobot; i++)
      interfaces[i].reset(new T(cartControllers[i]));
  }
};

#endif  // CONTROL_H