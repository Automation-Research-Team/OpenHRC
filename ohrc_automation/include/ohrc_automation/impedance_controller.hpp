#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/cart_controller.hpp"

class ImpedanceController {
  std::shared_ptr<CartController> controller;
  std::mutex mtx_imp;
  ros::Subscriber targetSubscriber;
  ros::Publisher RespawnReqPublisher;

  std::vector<Affine3d> _targetPoses;
  Affine3d restPose;

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

  enum class TaskState {
    Initial,
    OnGoing,
    Success,
    Fail,
  } taskState = TaskState::Initial;

  void getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK);
  ImpParam getImpParam(const ImpCoeff& impCoeff);
  Affine3d getNextTarget(const ImpedanceController::TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx, int& nextTargetIdx);

  TaskState updataTaskState(const VectorXd& delta_x, const int targetIdx);
  VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam);

  void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);

public:
  ImpedanceController(std::shared_ptr<CartController> controller);
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist);
};

#endif  // IMPEDANCE_CONTROLLER_HPP
