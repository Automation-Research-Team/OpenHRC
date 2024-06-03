#include "ohrc_shared_control/marker_teleoperation_cart_trajectory.hpp"

#include "ohrc_control/single_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_teleoperation_cart_trajectory");
  SingleInterface<MarkerTeleoperationCartTrajectory> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}