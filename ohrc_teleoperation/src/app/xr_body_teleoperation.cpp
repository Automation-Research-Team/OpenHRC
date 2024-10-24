#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/xr_body_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleInterface<XrBodyInterface>>();
  interface->control();

  return 0;
}