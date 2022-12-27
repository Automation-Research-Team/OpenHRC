#include "ohrc_control/multi_test_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_test_teleoperation");
  MultiTestController MultiTestController;
  if (MultiTestController.control() < 0)
    ROS_ERROR("End by some fails");
}