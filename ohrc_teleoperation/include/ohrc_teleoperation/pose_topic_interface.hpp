#ifndef POSE_TOPIC_INTERFACE_HPP
#define POSE_TOPIC_INTERFACE_HPP

#include <geometry_msgs/msg/pose.hpp>

#include "ohrc_control/interface.hpp"
#include "ohrc_msgs/msg/state.hpp"

class PoseTopicInterface : public Interface {
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose;
  geometry_msgs::msg::Pose _pose;

  double k_trans = 1.0;

protected:
  bool _flagTopic = false;

  Affine3d T;
  Matrix3d R;
  bool isFirst = true;

  std::mutex mtx_topic;

  ohrc_msgs::msg::State state;
  KDL::Frame prevPoses;

  void cbPose(const geometry_msgs::msg::Pose::SharedPtr msg);
  // virtual void modifyTargetState(geometry?::State& state){};
  virtual void setSubscriber();
  void setPoseFromTwistMsg(const geometry_msgs::msg::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist);

public:
  using Interface::Interface;
  void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  void resetInterface() override;
};

#endif  // POSE_TOPIC_INTERFACE_HPP