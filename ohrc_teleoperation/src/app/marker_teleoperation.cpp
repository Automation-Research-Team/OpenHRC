#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  SingleInterface<MarkerInterface> interface;
  std::cout << "aaa" << std::endl;
  if (interface.control() < 0)
    std::cout << "End by some fails" << std::endl;
}