#include "ohrc_control/my_ik.hpp"

namespace MyIK {

using namespace Eigen;

MyIK::MyIK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _eps, Affine3d T_base_world, SolveType _type)
  : initialized(false), eps(_eps), T_base_world(T_base_world), solvetype(_type) {
  ros::NodeHandle nh("~");

  urdf::Model robot_model;
  std::string xml_string;

  std::string urdf_xml, full_urdf_xml;
  nh.param("urdf_xml", urdf_xml, URDF_param);
  nh.searchParam(urdf_xml, full_urdf_xml);

  ROS_DEBUG_NAMED("my_ik", "Reading xml file from parameter server");
  if (!nh.getParam(full_urdf_xml, xml_string)) {
    ROS_FATAL_NAMED("my_ik", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return;
  }

  nh.param(full_urdf_xml, xml_string, std::string());
  robot_model.initString(xml_string);

  ROS_DEBUG_STREAM_NAMED("my_ik", "Reading joints and links from URDF");

  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    ROS_FATAL("Failed to extract kdl tree from xml robot description");

  if (!tree.getChain(base_link, tip_link, chain))
    ROS_FATAL("Couldn't find chain %s to %s", base_link.c_str(), tip_link.c_str());

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  nJnt = chain.getNrOfJoints();
  lb.resize(nJnt);
  ub.resize(nJnt);
  vb.resize(nJnt);

  double mergin = 3.0 / 180.0 * M_PI;

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i) {
    joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      joint_num++;
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS) {
        if (joint->safety) {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        } else {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      } else {
        hasLimits = 0;
      }
      if (hasLimits) {
        lb(joint_num - 1) = lower + mergin;
        ub(joint_num - 1) = upper - mergin;
      } else {
        lb(joint_num - 1) = std::numeric_limits<float>::lowest();
        ub(joint_num - 1) = std::numeric_limits<float>::max();
      }
      ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));

      float vlimit;
      int hasVLimits;
      if (joint->type != urdf::Joint::CONTINUOUS) {
        vlimit = joint->limits->velocity;
        hasVLimits = 1;
      } else {
        hasVLimits = 0;
      }
      if (hasVLimits) {
        vb(joint_num - 1) = vlimit;
      } else {
        vb(joint_num - 1) = std::numeric_limits<float>::max();
      }
      ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint " << joint->name << " " << vb(joint_num - 1));
    }
  }

  initialize();
}

MyIK::MyIK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _eps, Affine3d T_base_world, SolveType _type)
  : initialized(false), chain(_chain), lb(_q_min), ub(_q_max), eps(_eps), T_base_world(T_base_world), solvetype(_type) {
  initialize();
}

void MyIK::initialize() {
  assert(nJnt == lb.data.size());
  assert(nJnt == ub.data.size());

  jacsolver.reset(new KDL::ChainJntToJacSolver(chain));
  fksolver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  // nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain, lb, ub, maxtime, eps, NLOPT_IK::SumSq));
  // iksolver.reset(new KDL::ChainIkSolverPos_TL(chain, lb, ub, maxtime, eps, true, true));

  for (uint i = 0; i < chain.segments.size(); i++) {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos) {
      if (ub(types.size()) >= std::numeric_limits<float>::max() && lb(types.size()) <= std::numeric_limits<float>::lowest())
        types.push_back(BasicJointType::Continuous);
      else
        types.push_back(BasicJointType::RotJoint);
    } else if (type.find("Trans") != std::string::npos)
      types.push_back(BasicJointType::TransJoint);
  }

  assert(types.size() == lb.data.size());

  initialized = true;
}

VectorXd MyIK::getUpdatedJntLimit(const KDL::JntArray& q_cur, std::vector<double>& artificial_lower_limits, std::vector<double>& artificial_upper_limits, const double& dt) {
  std::vector<double> upper, lower, vel_limit;
  for (uint i = 0; i < nJnt; i++) {
    lower.push_back(this->lb.data(i));
    upper.push_back(this->ub.data(i));
    vel_limit.push_back(this->vb.data(i));
  }

  VectorXd x(nJnt);
  for (uint i = 0; i < x.size(); i++) {
    x[i] = q_cur(i);

    if (types[i] == BasicJointType::Continuous)
      continue;

    if (types[i] == BasicJointType::TransJoint) {
      x[i] = std::min(x[i], upper[i]);
      x[i] = std::max(x[i], lower[i]);
    }
  }

  progress = -3;

  artificial_lower_limits.resize(lower.size());
  artificial_upper_limits.resize(upper.size());

  for (uint i = 0; i < nJnt; i++) {
    if (types[i] == BasicJointType::Continuous) {
      artificial_lower_limits[i] = x[i] - 2 * M_PI;
      artificial_upper_limits[i] = x[i] + 2 * M_PI;
    } else if (types[i] == BasicJointType::TransJoint) {
      artificial_lower_limits[i] = lower[i];
      artificial_upper_limits[i] = upper[i];
    } else {
      artificial_lower_limits[i] = std::max(lower[i], x[i] - 2 * M_PI);
      artificial_upper_limits[i] = std::min(upper[i], x[i] + 2 * M_PI);
    }

    // velocity limitation
    artificial_lower_limits[i] = std::max(artificial_lower_limits[i], x[i] - vel_limit[i] * dt);
    artificial_upper_limits[i] = std::min(artificial_upper_limits[i], x[i] + vel_limit[i] * dt);
    // std::cout << artificial_lower_limits[i] << ", " << artificial_upper_limits[i] << std::endl;
  }

  return x;
}

VectorXd MyIK::getUpdatedJntVelLimit(const KDL::JntArray& q_cur, std::vector<double>& lower_vel_limits, std::vector<double>& upper_vel_limits, const double& dt) {
  std::vector<double> lower_limits, upper_limits;
  VectorXd x = getUpdatedJntLimit(q_cur, lower_limits, upper_limits, dt);

  lower_vel_limits.resize(nJnt);
  upper_vel_limits.resize(nJnt);
  for (uint i = 0; i < nJnt; i++) {
    lower_vel_limits[i] = std::min((lower_limits[i] - x[i]) / dt, 0.0);
    upper_vel_limits[i] = std::max((upper_limits[i] - x[i]) / dt, 0.0);
    // std::cout << lower_vel_limits[i] << " < x [" << i << "] < " << upper_vel_limits[i] << std::endl;
  }

  // std::cout << "---" << std::endl;

  return x;
}

VectorXd MyIK::getRandomJnt(const double& dt) {
  KDL::JntArray q(nJnt);
  std::vector<double> lower_limits, upper_limits;
  getUpdatedJntLimit(q, lower_limits, upper_limits, dt);

  VectorXd q_(nJnt);
  for (int i = 0; i < nJnt; i++) {
    std::mt19937 mt{ std::random_device{}() };

    std::uniform_real_distribution<> dist(lower_limits[i], upper_limits[i]);

    q_(i) = dist(mt);
  }
  return q_;
}

VectorXd MyIK::getRandomJntVel(const double& dt) {
  KDL::JntArray q(nJnt);
  std::vector<double> lower_limits, upper_limits;
  getUpdatedJntVelLimit(q, lower_limits, upper_limits, dt);

  VectorXd dq_(nJnt);
  for (int i = 0; i < nJnt; i++) {
    std::mt19937 mt{ std::random_device{}() };

    std::uniform_real_distribution<> dist(lower_limits[i], upper_limits[i]);

    dq_(i) = dist(mt);
  }

  return dq_;
}

int MyIK::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const double& dt) {
  q_out = q_init;

  if (!initialized) {
    ROS_ERROR("TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
    return -1;
  }

  boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;

  if (q_init.data.size() != types.size()) {
    ROS_ERROR_THROTTLE(1.0, "IK seeded with wrong number of joints.  Expected %d but got %d", (int)types.size(), (int)q_init.data.size());
    return -3;
  }

  std::vector<double> artificial_lower_limits, artificial_upper_limits;
  VectorXd x = getUpdatedJntLimit(q_init, artificial_lower_limits, artificial_upper_limits, dt);

  KDL::Jacobian jac(nJnt);
  KDL::JntArray q(nJnt);

  KDL::Frame p;
  VectorXd e(6);
  Affine3d T_d, T;
  tf::transformKDLToEigen(p_in, T_d);
  double time_left;
  double alpha = 0.01;

  VectorXd w = VectorXd::Ones(nJnt);
  for (int i = 1; i < nJnt; i++)
    w(i) = w(i - 1) * 3.0;
  MatrixXd W = w.asDiagonal();
  // VectorXd w(6);
  // w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;
  // MatrixXd W = w.asDiagonal();

  double w_n = 1.0e-3;

  static VectorXd x_rest = x;

  while (ros::ok()) {
    q.data = x;

    JntToJac(q, jac);
    JntToCart(q, p);
    tf::transformKDLToEigen(p, T);

    MatrixXd J = jac.data;
    e = getCartError(T, T_d);

    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    time_left = dt - diff.total_nanoseconds() * 1.0e-9;
    if (time_left < 0.)
      return -1;

    // ROS_INFO_STREAM(e.transpose() * e);
    if (e.dot(e) < eps)
      break;

    // std::cout << e.transpose() << std::endl;

    // Compute pseudo-inverse Jacobian
    MatrixXd J_pinv, J_w_pinv;
    // J_pinv = (J.transpose() * J).inverse() * J.transpose();
    // J_pinv = (J.transpose() * W.inverse() * J).inverse() * J.transpose() * W.inverse();  // weighted

    double gamma = 0.5 * e.transpose() * W * e + w_n;
    // double gamma = 0.5 * e.transpose() * e + w_n;                                                               // proposed by Sugihara-sensei
    J_w_pinv = (J.transpose() * W * J + gamma * MatrixXd::Identity(nJnt, nJnt)).inverse() * J.transpose() * W;  // weighted & Levenberg–Marquardt

    // Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // J_w_pinv = svd.matrixV() * (svd.singularValues().array() + ArrayXd::Ones(svd.singularValues().size()) * gamma).inverse().matrix().asDiagonal() * svd.matrixU().transpose();

    // Update
    x = x + alpha * J_w_pinv * e;
    // VectorXd k(nJnt);
    // k = x_rest - x;
    // MatrixXd J_null = (MatrixXd::Identity(nJnt, nJnt) - J_pinv * J);
    // x = x + alpha * (J_w_pinv * e + J_null * k / (k.norm() + 1.0));  // priority

    // check joint limit and nan error
    for (int i = 0; i < x.size(); i++) {
      if (std::isnan(x[i]))
        return -1;

      x[i] = std::min(std::max(x[i], artificial_lower_limits[i]), artificial_upper_limits[i]);
    }
  }

  // ROS_INFO_STREAM(diff.total_nanoseconds() * 1.0e-6);

  // std::cout << x.transpose() << std::endl;
  // std::cout << "---" << std::endl;

  q_out.data = x;

  return 1;
}

// VectorXd MyIK::getCartError(Affine3d T, Affine3d T_d) {
//   VectorXd e(6);
//   e.head(3) = T_d.translation() - T.translation();
//   // e.tail(3) = (Quaterniond(T_d.rotation()) * Quaterniond(T.rotation()).inverse()).vec();
//   // e.tail(3) = rotation_util::getQuaternionError(Quaterniond(T_d.rotation()), Quaterniond(T.rotation())); // including
//   // flip e.tail(3) = rotation_util::getQuaternionLogError(Quaterniond(T_d.rotation()), Quaterniond(T.rotation()));
//   e.tail(3) = rotation_util::getRotationMatrixError(T_d.rotation(), T.rotation());

//   return e;
// }

void MyIK::updateVelP(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel, VectorXd& e) {
  Matrix<double, 6, 1> v;
  tf::twistKDLToEigen(des_eff_vel, v);

  KDL::Frame p;
  JntToCart(q_cur, p);

  Affine3d T_d, T;
  tf::transformKDLToEigen(des_eff_pose, T_d);
  tf::transformKDLToEigen(p, T);
  e = getCartError(T, T_d);

  VectorXd kp = 3.0 * VectorXd::Ones(6);  // TODO: make this p gain as ros param
  kp.tail(3) = kp.tail(3) * 0.5 / M_PI;
  v = v + kp.asDiagonal() * e;

  tf::twistEigenToKDL(v, des_eff_vel);
}

int MyIK::CartToJntVel(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt) {
  boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  // int rc = CartToJntVel_pinv(q_cur, des_eff_pose, des_eff_vel, dq_des, dt);
  int rc = CartToJntVel_qp(q_cur, des_eff_pose, des_eff_vel, dq_des, dt);
  if (rc < 0)
    ros::shutdown();
  ROS_INFO_STREAM_THROTTLE(1.0, (boost::posix_time::microsec_clock::local_time() - start_time).total_nanoseconds() * 1.0e-9);

  return rc;
}

int MyIK::CartToJntVel_pinv(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt) {
  KDL::Jacobian jac(nJnt);
  JntToJac(q_cur, jac);

  MatrixXd J = jac.data;

  VectorXd w = VectorXd::Ones(6);
  w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;
  MatrixXd W = w.asDiagonal();

  Affine3d T_d, T;
  KDL::Frame p;
  JntToCart(q_cur, p);

  tf::transformKDLToEigen(des_eff_pose, T_d);
  tf::transformKDLToEigen(p, T);
  VectorXd e(6);
  e = getCartError(T, T_d);
  MatrixXd J_pinv, J_w_pinv;
  double w_n = 1.0e-4;
  double gamma = 0.5 * e.transpose() * W * e + w_n;  // proposed by Sugihara-sensei

  // J_w_pinv = (J.transpose() * W.inverse() * J + gamma * MatrixXd::Identity(6, 6)).inverse() * J.transpose() * W.inverse();  // weighted & Levenberg–Marquardt
  J_w_pinv =
      (J.transpose() * W * J + gamma * MatrixXd::Identity(nJnt, nJnt)).inverse() * J.transpose() * W;  // weighted & Levenberg–Marquardt // TODO: should be solved with SVD method?

  Matrix<double, 6, 1> dp;
  tf::twistKDLToEigen(des_eff_vel, dp);
  // dp.tail(3) = Vector3d::Zero();

  VectorXd kp = 1.0 * VectorXd::Ones(6);  // TODO: make this ros param
  J_pinv = (J.transpose() * J).inverse() * J.transpose();
  VectorXd dV = VectorXd::Zero(nJnt);

  // static ros::Time t0 = ros::Time::now();
  // double t = (ros::Time::now() - t0).toSec();

  // dV(0) = -(q_cur.data(0) - (0.2 + 0.2 * sin(2.0 * M_PI * 0.1 * t)));
  static VectorXd q_init = q_cur.data;
  dV = -(q_cur.data - q_init);
  // double eps_rest = ((MatrixXd::Identity(nJnt, nJnt) - J_w_pinv * J) * dV).norm();
  // double eps = 10.0 / (1.0 + eps_rest);

  static double maxManipulability = sqrt((J * J.transpose()).determinant());
  double curMani = sqrt((J * J.transpose()).determinant());

  if (maxManipulability < curMani)
    maxManipulability = curMani;

  double ratioMani = (curMani / maxManipulability);

  VectorXd k = 10.0 * ratioMani * ratioMani * ratioMani * ratioMani * dV;  // TODO: this is not good implementation...
  VectorXd dq = J_w_pinv * (dp + kp.asDiagonal() * getCartError(T, T_d)) + (MatrixXd::Identity(nJnt, nJnt) - J_w_pinv * J) * k;
  // VectorXd dq = -(MatrixXd::Identity(nJnt, nJnt) - J_w_pinv * J) * k;
  // std::cout << ratioMani << std::endl;
  // std::cout << "---" << std::endl;

  // check joint velocity limit
  std::vector<double> lower_vel_limits, upper_vel_limits;
  getUpdatedJntVelLimit(q_cur, lower_vel_limits, upper_vel_limits, dt);
  for (int i = 0; i < dq.size(); i++) {
    if (std::isnan(dq[i]))
      return -1;
    dq[i] = std::min(std::max(dq[i], lower_vel_limits[i]), upper_vel_limits[i]);
  }
  dq_des.data = dq;

  // std::cout << "--" << std::endl;

  return 1;
}

int MyIK::CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Twist& des_eff_vel, const VectorXd& e, KDL::JntArray& dq_des, const double& dt) {
  KDL::Jacobian jac(nJnt);
  JntToJac(q_cur, jac);
  MatrixXd J = jac.data;

  Matrix<double, 6, 1> v;
  tf::twistKDLToEigen(des_eff_vel, v);

  VectorXd w = VectorXd::Ones(6);
  w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;

  double w_n = 1.0e-5;  // this leads dq -> 0 witch is conflict with additonal task
  double gamma = 0.5 * e.transpose() * w.asDiagonal() * e + w_n;

  VectorXd dV = VectorXd::Zero(nJnt);

  dV(0) = -(q_cur.data(0) - 0.);
  // static ros::Time t0 = ros::Time::now();
  // double t = (ros::Time::now() - t0).toSec();
  // dV(0) = -(q_cur.data(0) - (0.3 * sin(2.0 * M_PI * 0.1 * t)));

  MatrixXd S = MatrixXd::Zero(1, nJnt);  // selection matrix
  S(0) = 1.0;

  std::vector<double> w_h = { 1.0e5, 1.0e2 };
  std::vector<MatrixXd> H(w_h.size());
  H[0] = J.transpose() * J + gamma * MatrixXd::Identity(nJnt, nJnt);
  H[1] = S.transpose() * S;
  std::vector<VectorXd> g(w_h.size());
  g[0] = v.transpose() * J;
  g[1] = dV.transpose() * S.transpose() * S;

  // allocate QP problem matrices and vectores
  SparseMatrix<double> hessian = std_utility::weightedSum(w_h, H).sparseView();
  VectorXd gradient = -std_utility::weightedSum(w_h, g);

  std::vector<double> lower_vel_limits, upper_vel_limits;
  getUpdatedJntVelLimit(q_cur, lower_vel_limits, upper_vel_limits, dt);
  Map<VectorXd> lowerBound(&lower_vel_limits[0], nJnt);
  Map<VectorXd> upperBound(&upper_vel_limits[0], nJnt);
  SparseMatrix<double> linearMatrix(nJnt, nJnt);  // TODO: is it better to separate velocity and joint limit?
  linearMatrix.setIdentity();

  // TODO: update variables. this seems to be difficult with OSQP since this library is for sparse QP optimization. When updating hessian matrix and changing its sparse form, the
  // function init the solver instanse. At that time, bounds is removed and failed to pass the bounds check. Other QP solver for dense problem would be a solusion. Actually, this
  // propblem matrix is not sparse.
  OsqpEigen::Solver qpSolver;
  qpSolver.settings()->setVerbosity(false);
  qpSolver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  qpSolver.data()->setNumberOfVariables(nJnt);
  qpSolver.data()->setNumberOfConstraints(nJnt);

  if (!qpSolver.data()->setHessianMatrix(hessian))
    return -1;
  if (!qpSolver.data()->setGradient(gradient))
    return -1;
  if (!qpSolver.data()->setLinearConstraintsMatrix(linearMatrix))
    return -1;
  if (!qpSolver.data()->setBounds(lowerBound, upperBound))
    return -1;

  // instantiate the solver
  if (!qpSolver.initSolver())
    return -1;

  // solve the QP problem
  OsqpEigen::ErrorExitFlag a = qpSolver.solveProblem();
  if (a != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << (int)a << "\n";
    return -1;
  }

  // get the controller input
  dq_des.data = qpSolver.getSolution();

  return 1;
}

int MyIK::CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt) {
  KDL::Twist des_eff_vel_ = des_eff_vel;
  VectorXd e;
  updateVelP(q_cur, des_eff_pose, des_eff_vel_, e);
  return CartToJntVel_qp(q_cur, des_eff_vel_, e, dq_des, dt);
}

int MyIK::CartToJntVel_qp_manipOpt(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt,
                                   const MatrixXd& userManipU) {
  KDL::Jacobian jac(nJnt);
  JntToJac(q_cur, jac);
  MatrixXd J = jac.data;

  KDL::Frame p;
  JntToCart(q_cur, p);

  Affine3d T_d, T;
  tf::transformKDLToEigen(des_eff_pose, T_d);
  tf::transformKDLToEigen(p, T);
  VectorXd e = getCartError(T, T_d);

  Matrix<double, 6, 1> v;
  tf::twistKDLToEigen(des_eff_vel, v);

  VectorXd kp = 3.0 * VectorXd::Ones(6);  // TODO: make this p gain as ros param
  v = v + kp.asDiagonal() * e;

  VectorXd w = VectorXd::Ones(6);
  w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;

  double w_n = 1.0e-5;  // this leads dq -> 0 witch is conflict with additonal task
  double gamma = 0.5 * e.transpose() * w.asDiagonal() * e + w_n;

  VectorXd dV = VectorXd::Zero(nJnt);

  dV(0) = -(q_cur.data(0) - 0.);
  // static ros::Time t0 = ros::Time::now();
  // double t = (ros::Time::now() - t0).toSec();
  // dV(0) = -(q_cur.data(0) - (0.3 * sin(2.0 * M_PI * 0.1 * t)));

  MatrixXd S = MatrixXd::Ones(1, nJnt);  // selection matrix
  // S(0) = 1.0;

  Eigen::JacobiSVD<MatrixXd> svd(J.block(0, 0, 3, nJnt), Eigen::ComputeThinU | Eigen::ComputeThinV);
  // VectorXd s = svd.singularValues();
  MatrixXd U = svd.matrixU();

  // make U as rotation matirix in right hand coordicate
  if (U.determinant() < 0)
    U.col(2) = -U.col(2);

  double e_manip = rotation_util::getRotationMatrixError(U, userManipU).norm();

  double delta = 0.01;
  for (int i = 0; i < nJnt; i++) {
    KDL::JntArray q_cur2 = q_cur;
    q_cur2.data(i) = q_cur2.data(i) + delta;
    KDL::Jacobian jac2(nJnt);
    JntToJac(q_cur2, jac2);
    MatrixXd J2 = jac2.data;

    Eigen::JacobiSVD<MatrixXd> svd2(J2.block(0, 0, 3, nJnt), Eigen::ComputeThinU | Eigen::ComputeThinV);
    // VectorXd s = svd.singularValues();
    MatrixXd U2 = svd2.matrixU();

    if (U2.determinant() < 0)
      U2.col(2) = -U2.col(2);

    double e_manip2 = rotation_util::getRotationMatrixError(U2, userManipU).norm();

    dV(i) = (e_manip2 - e_manip) / delta;
  }

  std::vector<double> w_h = { 1.0e5, 1.0e1 };
  std::vector<MatrixXd> H(w_h.size());
  H[0] = J.transpose() * J + gamma * MatrixXd::Identity(nJnt, nJnt);
  H[1] = S.transpose() * S;
  std::vector<VectorXd> g(w_h.size());
  g[0] = v.transpose() * J;
  g[1] = dV.transpose() * S.transpose() * S;

  // allocate QP problem matrices and vectores
  SparseMatrix<double> hessian = std_utility::weightedSum(w_h, H).sparseView();
  VectorXd gradient = -std_utility::weightedSum(w_h, g);

  std::vector<double> lower_vel_limits, upper_vel_limits;
  getUpdatedJntVelLimit(q_cur, lower_vel_limits, upper_vel_limits, dt);
  Map<VectorXd> lowerBound(&lower_vel_limits[0], nJnt);
  Map<VectorXd> upperBound(&upper_vel_limits[0], nJnt);
  SparseMatrix<double> linearMatrix(nJnt, nJnt);  // TODO: is it better to separate velocity and joint limit?
  linearMatrix.setIdentity();

  // TODO: update variables. this seems to be difficult with OSQP since this library is for sparse QP optimization. When updating hessian matrix and changing its sparse form, the
  // function init the solver instanse. At that time, bounds is removed and failed to pass the bounds check. Other QP solver for dense problem would be a solusion. Actually, this
  // propblem matrix is not sparse.
  OsqpEigen::Solver qpSolver;
  qpSolver.settings()->setVerbosity(false);
  qpSolver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  qpSolver.data()->setNumberOfVariables(nJnt);
  qpSolver.data()->setNumberOfConstraints(nJnt);

  if (!qpSolver.data()->setHessianMatrix(hessian))
    return -1;
  if (!qpSolver.data()->setGradient(gradient))
    return -1;
  if (!qpSolver.data()->setLinearConstraintsMatrix(linearMatrix))
    return -1;
  if (!qpSolver.data()->setBounds(lowerBound, upperBound))
    return -1;

  // instantiate the solver
  if (!qpSolver.initSolver())
    return -1;

  // solve the QP problem
  OsqpEigen::ErrorExitFlag a = qpSolver.solveProblem();
  if (a != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << (int)a << "\n";
    return -1;
  }

  // get the controller input
  dq_des.data = qpSolver.getSolution();

  return 1;
}

visualization_msgs::Marker MyIK::getManipulabilityMarker(const KDL::JntArray q_cur) {
  KDL::Jacobian jac(nJnt);
  JntToJac(q_cur, jac);
  MatrixXd J = jac.data.block(0, 0, 3, nJnt);

  KDL::Frame p;
  JntToCart(q_cur, p);
  Affine3d T;
  tf::transformKDLToEigen(p, T);

  Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  VectorXd s = svd.singularValues();
  MatrixXd U = svd.matrixU();

  // make U as rotation matirix in right hand coordicate
  if (U.determinant() < 0)
    U.col(2) = -U.col(2);

  Quaterniond q = Quaterniond(Matrix3d(U)).normalized();

  visualization_msgs::Marker manipuMarker;
  manipuMarker.header.frame_id = "test";
  manipuMarker.header.stamp = ros::Time::now();
  manipuMarker.id = 0;
  manipuMarker.type = visualization_msgs::Marker::SPHERE;
  manipuMarker.action = visualization_msgs::Marker::ADD;
  manipuMarker.pose.position = tf2::toMsg(Vector3d(T.translation()));
  manipuMarker.pose.orientation = tf2::toMsg(q);
  manipuMarker.color.a = 0.5;  // Don't forget to set the alpha!
  manipuMarker.color.r = 1.0;
  manipuMarker.color.g = 0.0;
  manipuMarker.color.b = 0.0;
  tf2::toMsg(Vector3d(s) * 0.5, manipuMarker.scale);

  return manipuMarker;
}

}  // namespace MyIK