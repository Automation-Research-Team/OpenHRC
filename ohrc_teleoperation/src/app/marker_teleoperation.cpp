#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleInterface<MarkerInterface>>();
  interface->control();

  rclcpp::shutdown();
  return 0;
}