#ifndef MARKER_TELEOPERATION_CART_TRAJECTROY_HPP
#define MARKER_TELEOPERATION_CART_TRAJECTROY_HPP

#include "ohrc_shared_control/switching_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"
#include "ohrc_automation/cart_trajectory_controller.hpp"

class MarkerTeleoperationCartTrajectory: virtual public SwitchingInterface<MarkerInterface, CartTrajectoryController>{

public:
    MarkerTeleoperationCartTrajectory(const std::shared_ptr<CartController> controller) : SwitchingInterface(controller), MarkerInterface(controller), CartTrajectoryController(controller), Interface(controller){
  }
};


#endif //MARKER_TELEOPERATION_CART_TRAJECTROY_HPP