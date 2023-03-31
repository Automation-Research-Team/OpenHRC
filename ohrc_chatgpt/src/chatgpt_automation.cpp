#include "ohrc_control/single_interface.hpp"
#include "ohrc_chatgpt/move_end_effector3.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_twist_topic_teleoperation");

  SingleInterface<MoveEndEffector> interface;
  if (interface.control() < 0) ROS_ERROR("End by some fails");
}
