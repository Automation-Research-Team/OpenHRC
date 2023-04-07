#ifndef MY_IK_HPP
#define MY_IK_HPP

#include <eigen_conversions/eigen_kdl.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <thread>

#include "OsqpEigen/OsqpEigen.h"
#include "ohrc_common/utility.h"

namespace MyIK {

enum SolveType { Pure };
enum BasicJointType { RotJoint, TransJoint, Continuous };

class MyIK {
  bool initialized, enableSelfCollisionAvoidance = false;

  KDL::Chain chain;
  KDL::JntArray lb, ub, vb;

  double eps;
  // double maxtime;
  SolveType solvetype;

  std::vector<BasicJointType> types;

  std::mutex mtx_;
  std::vector<KDL::JntArray> solutions;
  std::vector<std::pair<double, uint> > errors;

  // std::thread task1, task2;
  KDL::Twist bounds;

  std::vector<double> best_x;
  int progress;
  bool aborted;

  unsigned int nJnt;

  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fkVelSolver;

  Affine3d T_base_world;

  std::vector<std::string> nameJnt;
  std::vector<int> idxSegJnt;
  // OsqpEigen::Solver qpSolver;

  bool poseFeedbackDisabled = false;

  void initialize();

  int addSelfCollisionAvoidance(const KDL::JntArray& q_cur, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca);

public:
  VectorXd getUpdatedJntLimit(const KDL::JntArray& q_cur, std::vector<double>& artificial_lower_limits, std::vector<double>& artificial_upper_limits, const double& dt);
  VectorXd getUpdatedJntVelLimit(const KDL::JntArray& q_cur, std::vector<double>& lower_vel_limits, std::vector<double>& upper_vel_limits, const double& dt);

  MyIK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param = "/robot_description", double _eps = 1e-5,
       Affine3d T_base_world = Affine3d::Identity(), SolveType _type = Pure);
  MyIK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _eps = 1e-5, Affine3d T_base_world = Affine3d::Identity(),
       SolveType _type = Pure);

  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const double& dt = 1.0e5);
  int CartToJntVel(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt = 1.0e-5);
  int CartToJntVel_pinv(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt);
  int CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt);
  int CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Twist& des_eff_vel, const VectorXd& e, KDL::JntArray& dq_des, const double& dt);

  int CartToJntVel_qp_manipOpt(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt,
                               const MatrixXd& userManipU);

  void updateVelP(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel, const VectorXd& e);

  VectorXd getRandomJnt(const double& dt);
  VectorXd getRandomJntVel(const double& dt);

  inline int JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out) {
    return fksolver->JntToCart(q_in, p_out);
  }

  inline int JntToCart(const KDL::JntArray& q_in, const KDL::JntArray& dq_in, KDL::Frame& p_out, KDL::Twist& v_out) {
    KDL::JntArrayVel q_dq_in(q_in, dq_in);
    KDL::FrameVel p_v_out;
    int r = fkVelSolver->JntToCart(q_dq_in, p_v_out);
    p_out = p_v_out.GetFrame();
    v_out = p_v_out.GetTwist();
    return r;
  }

  inline int JntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac) {
    int n = q_in.data.size();
    if (jac.columns() < n)
      jac.resize(n);
    return jacsolver->JntToJac(q_in, jac);
  }

  inline int JntToJac(const KDL::JntArray& q_in, MatrixXd& J) {
    KDL::Jacobian jac;
    int r = JntToJac(q_in, jac);
    J = jac.data;
    return r;
  }

  // void setT_base_world(const Affine3d T_base_world) {
  //   this->T_base_world = T_base_world;
  // }
  inline Affine3d getT_base_world() {
    return this->T_base_world;
  }

  inline int JntVelToCartVel(const KDL::JntArray& q_in, const KDL::JntArray& dq_in, KDL::Twist& v_out) {
    // KDL::Jacobian jac;
    // JntToJac(q_in, jac);
    // Matrix<double, 6, 1> v;
    // v = jac.data * dq_in.data;
    // tf::twistEigenToKDL(v, v_out);
    // return 1;
    KDL::Frame p_out;
    return JntToCart(q_in, dq_in, p_out, v_out);
  }

  visualization_msgs::Marker getManipulabilityMarker(const KDL::JntArray q_cur);

  inline bool getKDLChain(KDL::Chain& chain_) {
    chain_ = chain;
    return initialized;
  }

  inline unsigned int getNJnt() {
    return nJnt;
  }

  inline void setNameJnt(std::vector<std::string> name) {
    this->nameJnt = name;
  }

  inline void setIdxSegJnt(std::vector<int> idx) {
    this->idxSegJnt = idx;
  }

  inline void enablePoseFeedback() {
    this->poseFeedbackDisabled = false;
  }
  inline void disablePoseFeedback() {
    this->poseFeedbackDisabled = true;
  }
};

inline VectorXd getCartError(const Affine3d& T, const Affine3d& T_d) {
  VectorXd e(6);
  e.head(3) = T_d.translation() - T.translation();
  // e.tail(3) = (Quaterniond(T_d.rotation()) * Quaterniond(T.rotation()).inverse()).vec();
  // e.tail(3) = rotation_util::getQuaternionError(Quaterniond(T_d.rotation()), Quaterniond(T.rotation())); // including
  // flip e.tail(3) = rotation_util::getQuaternionLogError(Quaterniond(T_d.rotation()), Quaterniond(T.rotation()));
  e.tail(3) = rotation_util::getRotationMatrixError(T_d.rotation(), T.rotation());

  return e;
}

inline VectorXd getCartError(const KDL::Frame& frame, const KDL::Frame& frame_d) {
  Affine3d T_d, T;
  tf::transformKDLToEigen(frame, T);
  tf::transformKDLToEigen(frame_d, T_d);

  return getCartError(T, T_d);
}

}  // namespace MyIK

#endif  // MY_IK_HPP