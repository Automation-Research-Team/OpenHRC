#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/pose_topic_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleInterface<PoseTopicInterface>>();
  interface->control();

  rclcpp::shutdown();
  return 0;
}