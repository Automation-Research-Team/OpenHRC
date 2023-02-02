#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include "ohrc_control/multi_cart_controller.hpp"

class ImpedanceController : public virtual MultiCartController {
  struct ImpParam {
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 3);
  };

  enum struct TaskState {
    OnGoing,
    Success,
    Fail,
  };

  ros::Subscriber subTargets;
  
};

#endif  // IMPEDANCE_CONTROLLER_HPP
