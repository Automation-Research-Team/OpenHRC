
#include "ohrc_automation/impedance_controller.hpp"
#include "ohrc_control/multi_cart_controller.hpp"

class ImpedanceControllerApp : virtual public MultiCartController {
  std::vector<std::shared_ptr<ImpedanceController>> impedanceController;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) override {
    impedanceController[controller->getIndex()]->updateTargetPose(pose, twist);
  }

public:
  ImpedanceControllerApp() {
    impedanceController.resize(nRobot);

    for (int i = 0; i < nRobot; i++)
      impedanceController[i].reset(new ImpedanceController(cartControllers[i]));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "impedance_automation");
  ImpedanceControllerApp controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}