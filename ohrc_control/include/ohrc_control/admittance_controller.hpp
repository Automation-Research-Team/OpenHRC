#ifndef ADMITTANCE_CONTOLLER_HPP
#define ADMITTANCE_CONTOLLER_HPP

#include <geometry_msgs/WrenchStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include "ohrc_control/cart_controller.hpp"
#include "std_srvs/Empty.h"

class AdmittanceController : public virtual CartController {
  ros::Subscriber subForce;
  ros::Publisher pubPoint;
  void cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  geometry_msgs::WrenchStamped _force;
  MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 3);
  Matrix3d I = Matrix3d::Zero(), Do = Matrix3d::Zero(), Ko = Matrix3d::Zero();

  // TransformUtility trans;
  Affine3d Tft_eff;

  bool initAdmittanceModel();
  bool initAdmittanceModel(std::vector<double> m, std::vector<double> d, std::vector<double> k, std::vector<double> i, std::vector<double> do_, std::vector<double> ko);

protected:
  virtual void initWithJnt(const KDL::JntArray& q_init) override;

  void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) override;
  void resetFt();

public:
  AdmittanceController();
};

#endif  // ADMITTANCE_CONTOLLER_HPP