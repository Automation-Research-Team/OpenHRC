#ifndef MULTI_TEST_CONTROLLER_HPP
#define MULTI_TEST_CONTROLLER_HPP

#include "ohrc_control/multi_cart_controller.hpp"

class MultiTestController : public virtual MultiCartController {
  void runLoopEnd() override;

  void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;



public:
  MultiTestController();
};

#endif  // MULTI_TEST_CONTROLLER_HPP