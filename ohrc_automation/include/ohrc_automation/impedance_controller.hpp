#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include "ohrc_control/multi_cart_controller.hpp"
#include "geometry_msgs/PoseArray.h"

class ImpedanceController : public virtual MultiCartController
{
  std::mutex mtx_imp;

  std::vector<Affine3d> targetPoses;
  Affine3d restPose;

  bool _targetUpdated = false;
  int targetIdx = -1;

  struct ImpParam
  {
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 3);
  } impParam;
  ros::Subscriber subTarget;

  enum class TaskState
  {
    Initial,
    OnGoing,
    Success,
    Fail,
  } taskState = TaskState::Initial;

  ImpParam getImpParam(const Vector3d& m, const Vector3d& k);
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  TaskState updataTaskState(const int targetIdx, CartController* controller);

  void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);

public:
  ImpedanceController();
};

#endif  // IMPEDANCE_CONTROLLER_HPP
