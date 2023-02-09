#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/state_topic_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_topic_teleoperation");
  SingleInterface<StateTopicInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}