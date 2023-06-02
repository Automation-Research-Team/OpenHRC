#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/interface.hpp"

class ImpedanceController : public virtual Interface {
  std::mutex mtx_imp;
  ros::Subscriber targetSubscriber;

  std::vector<Affine3d> _targetPoses;

  bool _targetUpdated = false;
  int targetIdx = -1, nextTargetIdx = -1;

  VectorXd x, xd;

  struct ImpParam {
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 6);
  } impParam;
  struct ImpCoeff {
    Vector3d m, d, k;
    std::vector<double> m_, d_, k_;
  };

  ros::Subscriber subTarget;

  void getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK);
  ImpParam getImpParam(const ImpCoeff& impCoeff);
  Affine3d getNextTarget(const TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx, int& nextTargetIdx);

  virtual TaskState updataTaskState(const VectorXd& delta_x, const int targetIdx);
  VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam);

  void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);

protected:
  ros::Publisher RespawnReqPublisher;
  int stack = 0;
  Affine3d restPose;

  virtual void setSubscriber();
  virtual bool updateImpedanceTarget(const VectorXd& x, VectorXd& xd);

public:
  using Interface::Interface;
  // ImpedanceController(std::shared_ptr<CartController> controller) : Interface(controller) {
  // }

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  virtual void initInterface() override;
};

#endif  // IMPEDANCE_CONTROLLER_HPP
