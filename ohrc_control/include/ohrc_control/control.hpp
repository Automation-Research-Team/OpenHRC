#ifndef CONTROL_H
#define CONTROL_H

#include "ohrc_control/interface.hpp"
#include "ohrc_control/multi_cart_controller.hpp"

class Control : virtual public MultiCartController {
protected:
  Control() {
    interfaces.resize(nRobot);
  }
};

#endif  // CONTROL_H