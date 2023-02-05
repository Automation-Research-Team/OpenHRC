#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include "geometry_msgs/PoseArray.h"
#include "ohrc_control/multi_cart_controller.hpp"

class ImpedanceController : public virtual MultiCartController {
  std::mutex mtx_imp;
  ros::Subscriber targetSubscriber;

  std::vector<Affine3d> _targetPoses;
  Affine3d restPose;

  bool _targetUpdated = false;
  int targetIdx = -1, nextTargetIdx = -1;

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
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  TaskState updataTaskState(const VectorXd& delta_x, CartController* controller);
  VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt);

  void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);

public:
  ImpedanceController();
};

#endif  // IMPEDANCE_CONTROLLER_HPP
