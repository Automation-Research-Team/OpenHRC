#ifndef POSITION_FEEDBACK_CONTROLLER_HPP
#define POSITION_FEEDBACK_CONTROLLER_HPP

#include "ohrc_control/interface.hpp"

class PositionFeedbackController : public virtual Interface {
  VectorXd x, xd;

  VectorXd PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist);
  VectorXd PIControl(const VectorXd& e, const KDL::Twist& twist);
  VectorXd adaptivePIControl(const rclcpp::Time t, const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist);
  VectorXd adaptivePIControl(const rclcpp::Time t, const VectorXd& e, const KDL::Twist& twist);
  VectorXd forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f);
  VectorXd forceFeedbackControl(const VectorXd& e, const VectorXd f);

  std::vector<double> t0_f;

public:
  using Interface::Interface;

  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
  virtual void initInterface() override;
};

#endif  // POSITION_FEEDBACK_CONTROLLER_HPP
