#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleInterface<TwistTopicInterface>>();
  interface->control();

  return 0;
}