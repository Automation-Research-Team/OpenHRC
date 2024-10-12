#ifndef POSE_TOPIC_INTERFACE_HPP
#define POSE_TOPIC_INTERFACE_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/interface.hpp"

class PoseTopicInterface : public Interface {
  ros::Subscriber subPose;
  geometry_msgs::Pose _pose;

  double k_trans = 1.0;

protected:
  bool _flagTopic = false;
  TransformUtility trans;

  Affine3d T;
  Matrix3d R;
  bool isFirst = true;

  std::mutex mtx_topic;

  ohrc_msgs::State state;
  KDL::Frame prevPoses;

  void cbPose(const geometry_msgs::Pose::ConstPtr& msg);
  // virtual void modifyTargetState(geometry?::State& state){};
  virtual void setSubscriber();
  void setPoseFromTwistMsg(const geometry_msgs::msg::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist);

public:
  using Interface::Interface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  void resetInterface() override;
};

#endif  // POSE_TOPIC_INTERFACE_HPP