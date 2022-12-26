
#include "ohrc_control/marker_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_teleoperation");
  MarkerController MarkerController;
  if (MarkerController.control() < 0)
    ROS_ERROR("End by some fails");
}