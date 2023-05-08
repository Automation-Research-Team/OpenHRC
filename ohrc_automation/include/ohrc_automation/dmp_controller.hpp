#ifndef DMP_CONTROLLER_HPP
#define DMP_CONTROLLER_HPP

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>

#include "ohrc_control/interface.hpp"

#define EIGEN_RUNTIME_NO_MALLOC  // Enable runtime tests for allocations

#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <set>
#include <string>

#include "dmp/Dmp.hpp"
#include "dmp/Trajectory.hpp"
#include "eigenutils/eigen_file_io.hpp"
#include "eigenutils/eigen_realtime_check.hpp"

using namespace std;
using namespace Eigen;
using namespace nlohmann;
using namespace DmpBbo;

class DmpController : public Interface {
  //   std::mutex mtx_imp;
  //   ros::Subscriber targetSubscriber;
  //   ros::Publisher RespawnReqPublisher;

  //   std::vector<Affine3d> _targetPoses;
  //   Affine3d restPose;

  //   bool _targetUpdated = false;
  //   int targetIdx = -1, nextTargetIdx = -1;

  //   VectorXd x, xd;
  //   int stack = 0;

  //   struct ImpParam {
  //     MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 6);
  //   } impParam;
  //   struct ImpCoeff {
  //     Vector3d m, d, k;
  //     std::vector<double> m_, d_, k_;
  //   };

  //   ros::Subscriber subTarget;

  //   void getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK);
  //   ImpParam getImpParam(const ImpCoeff& impCoeff);
  //   Affine3d getNextTarget(const TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx, int& nextTargetIdx);

  //   TaskState updataTaskState(const VectorXd& delta_x, const int targetIdx);
  //   VectorXd getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam);

  //   void cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg);
  //   Dmp* dmp;

public:
  using Interface::Interface;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
};

#endif  // DMP_CONTROLLER_HPP
