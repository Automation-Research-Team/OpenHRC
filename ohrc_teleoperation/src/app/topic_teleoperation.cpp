
#include "ohrc_control/topic_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "topic_teleoperation");
  TopicController TopicController;
  if (TopicController.control() < 0)
    ROS_ERROR("End by some fails");
}