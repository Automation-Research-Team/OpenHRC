#ifndef MULTI_MY_IK_HPP
#define MULTI_MY_IK_HPP

#include "ohrc_control/my_ik.hpp"

namespace MyIK {

class MultiMyIK {
  std::vector<std::vector<long long>> combsRobot, combosLink;

  int addCollisionAvoidance(const std::vector<Affine3d>& Ts, const std::vector<MatrixXd>& Js_, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_,
                            std::vector<MatrixXd>& A_ca);

  bool enableCollisionAvoidance = true;
  std::vector<double> w_h, init_w_h;

public:
  bool initialized;
  double eps;
  SolveType solvetype;

  int nState = 0;
  std::vector<int> iJnt;
  std::vector<std::shared_ptr<MyIK>> myIKs;
  int CartToJntVel_qp(const std::vector<KDL::JntArray>& q_cur, const std::vector<KDL::Frame>& des_eff_pose, const std::vector<KDL::Twist>& des_eff_vel,
                      std::vector<KDL::JntArray>& dq_des, const double& dt);
  int CartToJnt(const std::vector<KDL::JntArray>& q_init, const std::vector<KDL::Frame>& p_in, std::vector<KDL::JntArray>& q_out, const double& dt);
  const int nRobot;

  MultiMyIK(const std::vector<std::string>& base_link, const std::vector<std::string>& tip_link, const std::vector<std::string>& URDF_param,
            const std::vector<Affine3d>& T_base_world, const std::vector<std::shared_ptr<MyIK>>& myik_ptr, double _eps = 1e-5, SolveType _type = Pure);
  void resetRobotWeight();
  void setRobotWeight(int robotIndex, double rate);
};

};  // namespace MyIK

#endif  // MULTI_MY_IK_HPP
