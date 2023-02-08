
#include "ohrc_control/single_control.hpp"
#include "ohrc_teleoperation/joy_topic_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_topic_teleoperation");
  SingleControl<JoyTopicInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}