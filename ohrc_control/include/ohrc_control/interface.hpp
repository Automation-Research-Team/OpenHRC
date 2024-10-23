#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include "ohrc_control/cart_controller.hpp"

class Interface {
protected:
  // ros::NodeHandle n;
  const double dt;
  rclcpp::Node::SharedPtr node;

  // std::shared_ptr<TransformUtility> trans;

  std::shared_ptr<CartController> controller;

  // ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  TaskState taskState = TaskState::Initial;

  std::string targetName;
  double targetDistance = 0.0;
  VectorXd e;

  inline void reset() {
    controller->resetPose();
    controller->resetFt();
    resetInterface();
  }

  std::string stateTopicName = "/state", stateFrameId = "world";
  inline void getTopicAndFrameName(std::string DefaultStateTopicName, std::string DefaultStateFrameId) {
    // node->declare_parameter(controller->getRobotNs() + "topic_name", DefaultStateTopicName);
    // node->declare_parameter(controller->getRobotNs() + "frame_id", DefaultStateFrameId);

    // // if (!n.param(controller->getRobotNs() + "topic_name", stateTopicName, DefaultStateTopicName) &&
    // //     !n.param(controller->getRobotNs() + "frame_id", stateFrameId, DefaultStateFrameId))
    // if (!node->get_parameter(controller->getRobotNs() + "topic_name", stateTopicName) && !node->get_parameter(controller->getRobotNs() + "frame_id", stateFrameId))
    //   // ROS_WARN_STREAM("topic_name and/or frame_id is not explicitly configured. Use default: " << stateTopicName << " : " << stateFrameId);
    //   RCLCPP_WARN_STREAM(node->get_logger(), "topic_name and/or frame_id is not explicitly configured. Use default: " << stateTopicName << " : " << stateFrameId);
    // else
    //   // ROS_INFO_STREAM("topic_name: " << stateTopicName << "  frame_id: " << stateFrameId);
    //   RCLCPP_INFO_STREAM(node->get_logger(), "topic_name: " << stateTopicName << "  frame_id: " << stateFrameId);
    RclcppUtility::declare_and_get_parameter(node, "topic_name", DefaultStateTopicName, this->stateTopicName);
    RclcppUtility::declare_and_get_parameter(node, "frame_id", DefaultStateFrameId, this->stateFrameId);
  }

public:
  Interface(const std::shared_ptr<CartController>& controller) : node(controller->getNode()), dt(controller->dt) {
    this->controller = controller;
    // trans = std::make_shared<TransformUtility>(node);
  }

  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {};
  virtual void initInterface() {};
  virtual void resetInterface() {};
  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {};

  int targetIdx = -1, nCompletedTask = 0;
  bool blocked = false;

  inline std::string getTargetName() {
    return this->targetName;
  }

  inline double getTargetDistance() {
    return this->targetDistance;
  }

  inline TaskState getTaskState() {
    return this->taskState;
  }

  inline VectorXd getTargetError() {
    return this->e;
  }
};

#endif  // INTERFACE_HPP