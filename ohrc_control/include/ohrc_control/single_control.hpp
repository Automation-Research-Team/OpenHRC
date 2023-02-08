#ifndef SINGLE_CONTROL_H
#define SINGLE_CONTROL_H

#include "ohrc_control/control.hpp"

template <class T>
class SingleControl : virtual public Control {
public:
  SingleControl() {
    for (int i = 0; i < nRobot; i++)
      interfaces[i] = std::make_shared<T>(cartControllers[i]);
  }
};

#endif  // SINGLE_CONTROL_H