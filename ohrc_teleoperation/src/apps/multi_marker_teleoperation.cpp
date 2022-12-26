#include "ohrc_control/multi_marker_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_marker_teleoperation");
  MultiMarkerController MultiMarkerController;
  if (MultiMarkerController.control() < 0)
    ROS_ERROR("End by some fails");
}