#ifndef TWIST_TOPIC_ITNERFACE_HPP
#define TWIST_TOPIC_ITNERFACE_HPP

#include "geometry_msgs/Twist.h"
#include "ohrc_control/interface.hpp"

class TwistTopicInterface : public Interface {
protected:
  ros::Subscriber subTwist;
  Affine3d T_state_base;
  double k_trans = 1.0;

  TransformUtility trans;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  Affine3d T, T_start, T_state_start;
  bool isFirst = true;

  std::mutex mtx_twist;
  geometry_msgs::Twist _twist;

  double dt;

  ohrc_msgs::State state;

  std::string stateTopicName = "/cmd_vel", stateFrameId = "world";

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg);
  // virtual void modifyTargetState(geometry?::State& state){};

public:
  using Interface::Interface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  void resetInterface() override;
};

#endif  // TWIST_TOPIC_ITNERFACE_HPP