#ifndef MULTI_CART_COTNROLLER_HPP
#define MULTI_CART_COTNROLLER_HPP

#include <chrono>
#include <numeric>
#include <thread>

#include "ohrc_control/admittance_controller.hpp"
#include "ohrc_control/cart_controller.hpp"
#include "ohrc_control/hybrid_feedback_controller.hpp"
#include "ohrc_control/interface.hpp"
#include "ohrc_control/my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"
#include "ohrc_control/position_feedback_controller.hpp"

using namespace std::placeholders;
using namespace ohrc_control;
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  rclcpp::Node::SharedPtr node;

  std::vector<std::shared_ptr<MyIK::MyIK>> myik_ptr;

  rclcpp::TimerBase::SharedPtr control_timer;
  bool getRosParams(std::vector<std::string>& robots, std::vector<std::string>& hw_configs);
  void initMenbers(const std::vector<std::string> robots, const std::vector<std::string> hw_configs);
  void updateDesired();
  std::vector<KDL::Frame> desPose;
  std::vector<KDL::Twist> desVel;

  // ros::ServiceServer service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
  void resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response>& res);

  void publishState(const rclcpp::Time& time, const std::vector<KDL::Frame> curPose, const std::vector<KDL::Twist> curVel, const std::vector<KDL::Frame> desPose,
                    const std::vector<KDL::Twist> desVel);

  void controlLoop();
  virtual void starting();
  void stopping();
  void update(const rclcpp::Time& time, const rclcpp::Duration& period);

  enum class MFMode { Individual, Parallel, Cooperation, None } MFmode;
  enum class IKMode { Concatenated, Order, Parallel, None } IKmode;

  // std::vector<std::string> robots;
  // std::vector<std::string> hw_configs;
  int nRobot = 0;

  std::string root_frame;
  double freq = 500.0;
  double dt;
  rclcpp::Time t0;
  std::string date;

  // MyIK
  std::unique_ptr<MyIK::MyIK> multimyik_solver_ptr;

  ControllerType controller;
  PublisherType publisher;

  std::vector<int> manualInd, autoInd;

  std::vector<rclcpp::Time> prev_time;

  enum class PriorityType { Manual, Automation, Adaptation, None } priority;
  bool adaptation = false;
  void setPriority(PriorityType priority);
  void setPriority(std::vector<int> idx);
  void setPriority(int i);
  void setLowPriority(int i);
  void setHightLowPriority(int high, int low);

  // enum class AdaptationOption { Default, None } adaptationOption;
  std::string adaptationOption_;

  FeedbackMode feedbackMode;

  virtual void runLoopEnd() {};

  std::vector<std::shared_ptr<Interface>> baseControllers;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
    interfaces[controller->getIndex()]->updateTargetPose(this->get_clock()->now(), pose, twist);
    baseControllers[controller->getIndex()]->updateTargetPose(this->get_clock()->now(), pose, twist);
  }

  // void applyBaseControl(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
  //   baseControllers[controller->getIndex()]->updateTargetPose(pose, twist);
  // }

  // virtual void defineInterface() = 0;

  void initInterface(const std::shared_ptr<CartController>& controller) {
    interfaces[controller->getIndex()]->initInterface();
    baseControllers[controller->getIndex()]->initInterface();
  }

  void resetInterface(const std::shared_ptr<CartController>& controller) {
    interfaces[controller->getIndex()]->resetInterface();
    baseControllers[controller->getIndex()]->resetInterface();
  }

  void feedback(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
    interfaces[controller->getIndex()]->feedback(pose, twist);
  }

  virtual void preInterfaceProcess(std::vector<std::shared_ptr<Interface>> interfaces) {};

  virtual void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
    updateTargetPose(pose, twist, controller);
  };
  virtual void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
    updateTargetPose(pose, twist, controller);
  };
  // virtual void feedbackJnt(const KDL::JntArray& q_cur, const KDL::JntArray& q_des, std::shared_ptr<CartController> controller){};
  // virtual void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller){};

protected:
  virtual void defineInterface() {};
  std::vector<std::shared_ptr<Interface>> interfaces;
  std::vector<std::shared_ptr<CartController>> cartControllers;
  int getNRobot() {
    return nRobot;
  }

public:
  // rclcpp::executors::MultiThreadedExecutor exec;
  // std::vector<rclcpp::Node::SharedPtr> nodes;
  // rclcpp::Node::SharedPtr node;
  Controller();
  ~Controller();
  void control();
};

#endif  // MULTI_CART_COTNROLLER_HPP
