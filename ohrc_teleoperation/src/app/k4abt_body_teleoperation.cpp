#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/k4abt_body_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "k4abt_body_teleoperation");
  SingleInterface<K4abtBodyInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}