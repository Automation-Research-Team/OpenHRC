#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "ohrc_control/interface.hpp"

class ImpedanceController : public virtual Interface {
  ros::Subscriber targetSubscriber;

  bool _targetUpdated = false;
  // int targetIdx = -1,
  int nextTargetIdx = -1;

  VectorXd x, xd;

  struct ImpParam {
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 6);
  } impParam;
  struct ImpCoeff {
    Vector3d m, d, k;
    std::vector<double> m_, d_, k_;
  };

  ros::Subscriber subTarget;

  ros::Time t_start;

  void getParam();

  void getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK);
  ImpParam getImpParam(const ImpCoeff& impCoeff);
  ImpCoeff getImpCoeff();
  Affine3d getNextTarget(const TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx, int& nextTargetIdx);

  VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam);

  bool NormReachedCheck(const VectorXd& delta_x, const VectorXd& force, const double posThr, const double velThr, const double forceThr);

protected:
  std::mutex mtx_imp;
  ros::Publisher RespawnReqPublisher, targetDistPublisher;
  int stack = 0;
  Affine3d restPose;
  std::vector<Affine3d> _targetPoses;
  double timeLimit = 30.0, forceLimit = 100.0;
  double posThr = 0.01, velThr = 0.01, forceThr = -0.01;
  double posThr_r = 0.01, velThr_r = 0.01, forceThr_r = -0.01;
  bool repeat = true, restEverytime = true;

  virtual void setSubscriber();
  virtual bool updateImpedanceTarget(const VectorXd& x, VectorXd& xd);

  TaskState updataTaskState(const VectorXd& delta_x, const int targetIdx);
  virtual bool checkTargetReached(const VectorXd& delta_x, const VectorXd& force);
  virtual bool checkRestTargetReached(const VectorXd& delta_x, const VectorXd& force);

  virtual void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);

  inline ros::Duration getTrialTime() {
    return ros::Time::now() - t_start;
  }

public:
  using Interface::Interface;
  // ImpedanceController(std::shared_ptr<CartController> controller) : Interface(controller) {
  // }

  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  virtual void initInterface() override;
};

#endif  // IMPEDANCE_CONTROLLER_HPP
