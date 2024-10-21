
#include "ohrc_control/multi_cart_controller.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

class MultiInterface : virtual public Controller {
public:
  MultiInterface() {
    interfaces[0] = std::make_shared<MarkerInterface>(cartControllers[0]);
    interfaces[1] = std::make_shared<TwistTopicInterface>(cartControllers[1]);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_twist_topic_teleoperation");

  MultiInterface interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}