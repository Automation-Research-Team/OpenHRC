
#include "ohrc_control/control.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

// class TwistTopicInterfaceApp : virtual public Control<TwistTopicInterface> {
// public:
//   TwistTopicInterfaceApp() {
//     // for (int i = 0; i < nRobot; i++)
//     //   interfaces[i].reset(new TwistTopicInterface(cartControllers[i]));
//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_topic_teleoperation");
  Control<TwistTopicInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}