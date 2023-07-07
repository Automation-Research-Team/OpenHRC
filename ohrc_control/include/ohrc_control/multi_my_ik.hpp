#ifndef MULTI_MY_IK_HPP
#define MULTI_MY_IK_HPP

#include "ohrc_control/my_ik.hpp"

namespace MyIK {

class MultiMyIK {
  std::vector<std::vector<long long>> combsRobot, combosLink;

  int addCollisionAvoidance(const std::vector<Affine3d>& Ts, const std::vector<MatrixXd>& Js_, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_,
                            std::vector<MatrixXd>& A_ca);
  int addCollisionAvoidance(const std::vector<KDL::JntArray>& q_cur, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca);

  int calcCollisionAvoidance(int c0, int c1, const std::vector<std::vector<Vector3d>>& p_all, const std::vector<std::vector<KDL::Jacobian>>& J_all, double ds, double di,
                             double eta, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca);

  void getClosestPointLineSegments(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, double& as, double& bs);
  double getDistance(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, const double& as, const double& bs);
  Vector3d getVec(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, const double& as, const double& bs);

  bool enableCollisionAvoidance = true;
  std::vector<double> w_h, init_w_h;

  std::vector<KDL::JntArray> q_rest;

  int nAddObj = 0;
  Eigen::Matrix<double, Eigen::Dynamic, 1> primalVariable;

public:
  bool initialized = false;
  double eps;
  SolveType solvetype;

  int nState = 0;
  std::vector<int> iJnt;
  const std::vector<std::shared_ptr<MyIK>> myIKs;
  int CartToJntVel_qp(const std::vector<KDL::JntArray>& q_cur, const std::vector<KDL::Frame>& des_eff_pose, const std::vector<KDL::Twist>& des_eff_vel,
                      std::vector<KDL::JntArray>& dq_des, const double& dt);
  int CartToJnt(const std::vector<KDL::JntArray>& q_init, const std::vector<KDL::Frame>& p_in, std::vector<KDL::JntArray>& q_out, const double& dt);
  const int nRobot;

  MultiMyIK(const std::vector<std::string>& base_link, const std::vector<std::string>& tip_link, const std::vector<std::string>& URDF_param,
            const std::vector<Affine3d>& T_base_world, const std::vector<std::shared_ptr<MyIK>>& myik_ptr, double _eps = 1e-5, SolveType _type = Pure);
  void resetRobotWeight();
  void setRobotWeight(int robotIndex, double rate);

  void setqRest(const std::vector<KDL::JntArray>& q_rest);
};

};      // namespace MyIK

#endif  // MULTI_MY_IK_HPP
