#ifndef ADMITTANCE_CONTROLLER_HPP
#define ADMITTANCE_CONTROLLER_HPP

#include "ohrc_control/interface.hpp"

class AdmittanceController : public virtual Interface {
  VectorXd x, xd;

  struct ImpParam {
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 6);
  } impParam;
  struct ImpCoeff {
    Vector3d m, d, k;
    std::vector<double> m_, d_, k_;
  };

  void getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK);
  ImpParam getImpParam(const ImpCoeff& impCoeff);
  ImpCoeff getImpCoeff();

  VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam);

  Affine3d restPose;

public:
  using Interface::Interface;

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  virtual void initInterface() override;
};

#endif  // ADMITTANCE_CONTROLLER_HPP
