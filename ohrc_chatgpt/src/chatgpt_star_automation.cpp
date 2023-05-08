#include "ohrc_chatgpt/chatgpt_star_automation.hpp"

#include "ohrc_control/single_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_twist_topic_teleoperation");

  SingleInterface<MoveEndEffector> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}
