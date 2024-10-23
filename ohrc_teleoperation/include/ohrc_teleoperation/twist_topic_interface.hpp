#ifndef TWIST_TOPIC_ITNERFACE_HPP
#define TWIST_TOPIC_ITNERFACE_HPP

#include <geometry_msgs/msg/twist.hpp>

#include "ohrc_control/interface.hpp"

class TwistTopicInterface : virtual public Interface {
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTwist;
  geometry_msgs::msg::Twist _twist;

  double k_trans = 1.0;

protected:
  bool _flagTopic = false;


  Affine3d T;
  Matrix3d R;
  bool isFirst = true;

  std::mutex mtx_topic;

  ohrc_msgs::msg::State state;

  std::string stateTopicName = "/cmd_vel", stateFrameId = "world";

  void cbTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
  // virtual void modifyTargetState(geometry?::State& state){};
  virtual void setSubscriber();
  void setPoseFromTwistMsg(const geometry_msgs::msg::Twist& twist_msg, KDL::Frame& pos, KDL::Twist& twist);

public:
  using Interface::Interface;
  void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  void resetInterface() override;
};

#endif  // TWIST_TOPIC_ITNERFACE_HPP