#include "ohrc_control/control.hpp"
#include "ohrc_teleoperation/omega_interface.hpp"

// class OmegaInterfaceApp : virtual public Control<OmegaInterface> {
// public:
//   OmegaInterfaceApp() {
//     // for (int i = 0; i < nRobot; i++)
//     //   interfaces[i].reset(new OmegaInterface(cartControllers[i]));
//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_teleoperation");
  Control<OmegaInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}