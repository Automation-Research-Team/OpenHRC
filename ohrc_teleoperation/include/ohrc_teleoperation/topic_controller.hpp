#ifndef TOPIC_CONTROLLER_HPP
#define TOPIC_CONTROLLER_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/cart_controller.hpp"

class TopicController : public virtual CartController {
protected:
  ros::Subscriber subCmd;

  void cbCmd(const geometry_msgs::Pose::ConstPtr &cmd);

public:
  TopicController();
};

#endif  // TOPIC_CONTROLLER_HPP