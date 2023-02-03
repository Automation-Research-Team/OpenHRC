#ifndef TWIST_TOPIC_ITNERFACE_HPP
#define TWIST_TOPIC_ITNERFACE_HPP

#include "geometry_msgs/Twist.h"
#include "ohrc_control/multi_cart_controller.hpp"

class TwistTopicInterface : public virtual MultiCartController {
protected:
  ros::Subscriber subTwist;
  std::vector<Affine3d> T_state_base;
  double k_trans = 1.0;

  TransformUtility trans;

  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  struct s_updateManualTargetPose {
    Affine3d T, T_start, T_state_start;
    bool isFirst = true;
  };
  std::vector<s_updateManualTargetPose> s_updateManualTargetPoses;

  std::mutex mtx_twist;
  geometry_msgs::Twist _twist;

  ohrc_msgs::State state;

  std::string stateTopicName = "/cmd_vel", stateFrameId = "world";

  void starting() override;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg);
  // virtual void modifyTargetState(geometry?::State& state){};

  void resetInterface() override;

public:
  TwistTopicInterface();
};

#endif  // TWIST_TOPIC_ITNERFACE_HPP