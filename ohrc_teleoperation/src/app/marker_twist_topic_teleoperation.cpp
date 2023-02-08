
#include "ohrc_control/control.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

class MultiControl : virtual public Control {
public:
  MultiControl() {
    interfaces.resize(nRobot);
    interfaces[0] = std::make_shared<MarkerInterface>(cartControllers[0]);
    interfaces[1] = std::make_shared<TwistTopicInterface>(cartControllers[1]);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_teleoperation");

  MultiControl interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}