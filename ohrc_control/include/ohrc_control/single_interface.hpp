#ifndef SINGLE_INTERFACE_H
#define SINGLE_INTERFACE_H

#include "ohrc_control/multi_cart_controller.hpp"

template <class T>
class SingleInterface : virtual public Controller {
public:
  SingleInterface() {
    for (int i = 0; i < this->getNRobot(); i++)
      this->interfaces[i] = std::make_shared<T>(cartControllers[i]);
  }
};

#endif  // SINGLE_INTERFACE_H