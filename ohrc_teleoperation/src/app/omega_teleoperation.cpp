#include "ohrc_control/single_control.hpp"
#include "ohrc_teleoperation/omega_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_teleoperation");
  SingleControl<OmegaInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}