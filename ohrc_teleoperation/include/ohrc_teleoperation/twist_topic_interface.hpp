#ifndef TWIST_TOPIC_ITNERFACE_HPP
#define TWIST_TOPIC_ITNERFACE_HPP

#include "geometry_msgs/Twist.h"
#include "ohrc_control/interface.hpp"

class TwistTopicInterface : virtual public Interface {
  ros::Subscriber subTwist;
  geometry_msgs::Twist _twist;

  double k_trans = 1.0;

protected:
  bool _flagTopic = false;
  TransformUtility trans;

  Affine3d T;
  Matrix3d R;
  bool isFirst = true;

  std::mutex mtx_topic;

  ohrc_msgs::State state;

  std::string stateTopicName = "/cmd_vel", stateFrameId = "world";

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg);
  // virtual void modifyTargetState(geometry?::State& state){};
  virtual void setSubscriber();
  void setPoseFromTwistMsg(const geometry_msgs::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist);

public:
  using Interface::Interface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  void resetInterface() override;
};

#endif  // TWIST_TOPIC_ITNERFACE_HPP