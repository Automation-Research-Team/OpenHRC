#ifndef MULTI_CART_COTNROLLER_HPP
#define MULTI_CART_COTNROLLER_HPP

#include <numeric>
#include <thread>

#include "ohrc_control/cart_controller.hpp"
#include "ohrc_control/multi_my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"

using namespace ohrc_control;

class MultiCartController {
  bool getInitParam();
  void updateDesired();
  std::vector<KDL::Frame> desPose;
  std::vector<KDL::Twist> desVel;

  ros::ServiceServer service;
  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

protected:
  ros::NodeHandle nh;

  enum class MFMode { None, Individual, Parallel, Cooperation } MFmode;
  enum class IKMode { None, Concatenated, Order, Parallel } IKmode;

  std::vector<std::shared_ptr<CartController>> cartControllers;
  std::vector<std::string> robots;
  int nRobot = 0;

  std::string root_frame;
  double freq = 500.0;
  double dt = 0.002;
  ros::Time t0;
  std::string date;

  // MyIK
  std::unique_ptr<MyIK::MultiMyIK> multimyik_solver_ptr;

  ControllerType controller;
  PublisherType publisher;

  std::vector<int> manualInd, autoInd;

  enum class PriorityType { Manual, Automation, Adaptation, None } priority;
  bool adaptation = false;
  void setPriority(PriorityType priority);

  // enum class AdaptationOption { Default, None } adaptationOption;
  std::string adaptationOption_;

  virtual void runLoopEnd(){};
  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller){};
  virtual void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  virtual void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  virtual void feedbackJnt(const KDL::JntArray& q_cur, const KDL::JntArray& q_des, std::shared_ptr<CartController> controller){};
  virtual void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller){};

  virtual void resetInterface(std::shared_ptr<CartController> controller){};

  template <typename T>
  inline T getEnumParam(const std::string& key, T none, const std::string default_str, ros::NodeHandle n) {
    std::string s;
    if (!n.getParam(key, s)) {
      ROS_INFO_STREAM("Failed to get " << key << ", so" << default_str << "is automatically selected");
      s = default_str;
    }

    T mode = magic_enum::enum_cast<T>(s).value_or(none);
    if (mode == none) {
      ROS_FATAL_STREAM(key << " is configured as [" << s << "] and not correctly configured.");
      ros::shutdown();
    } else
      ROS_INFO_STREAM("Operation mode: " << s);

    return mode;
  }

public:
  MultiCartController();
  int control();
  virtual void starting();
  void stopping();
  void update(const ros::Time& time, const ros::Duration& period);
};

#endif  // MULTI_CART_COTNROLLER_HPP