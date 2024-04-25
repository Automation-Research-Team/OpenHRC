#ifndef FEEDBACK_CONTROLLER_HPP
#define FEEDBACK_CONTROLLER_HPP

#include "ohrc_control/interface.hpp"

class FeedbackController : public virtual Interface {
  VectorXd x, xd;

  VectorXd PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist);
  VectorXd forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f);

  std::vector<double> t0_f;
public:
  using Interface::Interface;

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
    virtual void initInterface() override;
};

#endif  // FEEDBACK_CONTROLLER_HPP
