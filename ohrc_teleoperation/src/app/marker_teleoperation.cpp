#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  SingleInterface<MarkerInterface> interface;
  if (interface.control() < 0)
    // ROS_ERROR("End by some fails");
    return 0;
}