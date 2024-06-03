#ifndef STATE_TOPIC_ITNERFACE_HPP
#define STATE_TOPIC_ITNERFACE_HPP

#include "ohrc_control/interface.hpp"
#include "ohrc_msgs/State.h"

class StateTopicInterface : public Interface {
protected:
  ros::Subscriber subState;
  Affine3d T_state_base;
  double k_trans = 1.0;

  TransformUtility trans;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  Affine3d T, T_start, T_state_start;
  bool isFirst = true;

  std::mutex mtx_state;
  ohrc_msgs::State _state;
  bool _flagTopic = false;

  void cbState(const ohrc_msgs::State::ConstPtr& msg);
  virtual void modifyTargetState(ohrc_msgs::State& state) {};

  virtual void setSubscriber();
  void getTargetState(const ohrc_msgs::State& state, KDL::Frame& pos, KDL::Twist& twist);

public:
  using Interface::Interface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
};

#endif  // STATE_TOPIC_ITNERFACE_HPP