#ifndef STATE_TOPIC_ITNERFACE_HPP
#define STATE_TOPIC_ITNERFACE_HPP

#include "ohrc_control/multi_cart_controller.hpp"
#include "ohrc_msgs/State.h"

class StateTopicInterface : public virtual MultiCartController {
protected:
  ros::Subscriber subState;
  std::vector<Affine3d> T_state_base;
  double k_trans = 1.0;

  TransformUtility trans;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  struct s_updateManualTargetPose {
    Affine3d T, T_start, T_state_start;
    bool isFirst = true;
  };
  std::vector<s_updateManualTargetPose> s_updateManualTargetPoses;

  std::mutex mtx_state;
  ohrc_msgs::State _state;

  std::string stateTopicName = "/state", stateFrameId = "world";

  void starting() override;
  void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;

  void cbState(const ohrc_msgs::State::ConstPtr& msg);

public:
  StateTopicInterface();
};

#endif  // STATE_TOPIC_ITNERFACE_HPP