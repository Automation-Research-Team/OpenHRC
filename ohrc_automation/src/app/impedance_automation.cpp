
#include "ohrc_automation/impedance_controller.hpp"
#include "ohrc_control/control.hpp"

// class ImpedanceControllerApp : virtual public Control<ImpedanceController> {
// public:
//   ImpedanceControllerApp() {
//     // for (int i = 0; i < nRobot; i++)
//     //   interfaces[i].reset(new ImpedanceController(cartControllers[i]));
//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "impedance_automation");
  Control<ImpedanceController> controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}