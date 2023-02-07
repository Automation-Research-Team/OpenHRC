
#include "ohrc_control/control.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

// class MarkerInterfaceApp : virtual public Control<MarkerInterface> {
// public:
//   MarkerInterfaceApp() {
//     // for (int i = 0; i < nRobot; i++)
//     //   interfaces[i].reset(new MarkerInterface(cartControllers[i]));
//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_teleoperation");
  Control<MarkerInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}