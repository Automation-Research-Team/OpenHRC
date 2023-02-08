
#include "ohrc_control/control.hpp"
#include "ohrc_teleoperation/state_topic_interface.hpp"

// class StateTopicInterfaceApp : virtual public Control<StateTopicInterface> {
// public:
//   StateTopicInterfaceApp() {
//     // for (int i = 0; i < nRobot; i++)
//     //   interfaces[i].reset(new StateTopicInterface(cartControllers[i]));
//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_topic_teleoperation");
  Control<StateTopicInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}