#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_topic_teleoperation");
  SingleInterface<TwistTopicInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}