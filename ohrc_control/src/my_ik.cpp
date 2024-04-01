#include "ohrc_control/my_ik.hpp"

namespace MyIK {

MyIK::MyIK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _eps, Affine3d T_base_world, SolveType _type)
  : nRobot(1), initialized(false), eps(_eps), T_base_world(T_base_world), solvetype(_type) {
  ros::NodeHandle nh("~");

  urdf::Model robot_model = model_utility::getURDFModel(URDF_param, nh);
  chain = model_utility::getKDLChain(robot_model, base_link, tip_link);

  nJnt = chain.getNrOfJoints();
  nSeg = chain.getNrOfSegments();
  lb.resize(nJnt);
  ub.resize(nJnt);
  vb.resize(nJnt);

  double mergin = 5.0 / 180.0 * M_PI;
  model_utility::getBounds(robot_model, chain, mergin, lb, ub, vb);

  if (!nh.param("self_collision_avoidance", enableSelfCollisionAvoidance, false)) {
    ROS_WARN_STREAM("self_collision_avoidance is not configured. Default is False");
  }
  initializeSingleRobot(chain);
}

MyIK::MyIK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _eps, Affine3d T_base_world, SolveType _type)
  : nRobot(1), initialized(false), lb(_q_min), ub(_q_max), eps(_eps), T_base_world(T_base_world), solvetype(_type) {
  initializeSingleRobot(_chain);
}

MyIK::MyIK(const std::vector<std::string>& base_link, const std::vector<std::string>& tip_link, const std::vector<Affine3d>& T_base_world,
           const std::vector<std::shared_ptr<MyIK>>& myik_ptr, double _eps, SolveType _type)
  : nRobot(base_link.size()), eps(_eps), solvetype(_type), myIKs(myik_ptr) {
  // get number of elements of the state vector and indeces corresponding to start of joint vector of each robot
  iJnt.resize(nRobot + 1, 0);
  for (int i = 0; i < nRobot; i++) {
    nState += myIKs[i]->getNJnt();
    iJnt[i + 1] = iJnt[i] + myIKs[i]->getNJnt();
  }

  // get combinations of robots
  if (nRobot > 1)
    combsRobot = std_utility::comb(nRobot, 2);

  // initialize Ik weights
  init_w_h.resize(nRobot + nAddObj, 1.0e4);
  w_h = init_w_h;

  // check all robots have been initialized
  int initializedRobot = 0;
  for (int i = 0; i < nRobot; i++)
    initializedRobot += (int)myIKs[i]->getInitialized();

  if (initializedRobot == nRobot)
    initialized = true;
}

void MyIK::initializeSingleRobot(const KDL::Chain& chain) {
  // std::cout << ub.data.transpose() << std::endl;
  assert(nJnt == lb.data.size());
  assert(nJnt == ub.data.size());

  jacsolver.reset(new KDL::ChainJntToJacSolver(chain));
  fksolver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  fkVelSolver.reset(new KDL::ChainFkSolverVel_recursive(chain));

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

  MyIK* a = this;
  myIKs.resize(1);
  myIKs[0] = std::shared_ptr<MyIK>(a);  // TODO: use shared_from_this();

  iJnt.resize(nRobot + 1, 0);
  for (int i = 0; i < nRobot; i++) {
    nState += myIKs[i]->getNJnt();
    iJnt[i + 1] = iJnt[i] + myIKs[i]->getNJnt();
  }

  init_w_h.resize(nRobot + nAddObj, 1.0e4);
  w_h = init_w_h;

  q_rest.resize(nRobot);
  q_rest[0] = KDL::JntArray(nJnt);  // TODO: set rest position with inital states

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

    // if (types[i] == BasicJointType::Continuous)
    //   continue;

    // if (types[i] == BasicJointType::TransJoint) {
    //   x[i] = std::min(x[i], upper[i]);
    //   x[i] = std::max(x[i], lower[i]);
    // }
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

  double mergin = 1.0e-3;
  for (uint i = 0; i < nJnt; i++) {
    lower_vel_limits[i] = std::min((lower_limits[i] - x[i]) / dt, mergin);
    upper_vel_limits[i] = std::max((upper_limits[i] - x[i]) / dt, -mergin);
  }

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
  std::vector<KDL::JntArray> q_out_ = std_utility::makeVector(q_out);
  int out = this->CartToJnt(std_utility::makeVector(q_init), std_utility::makeVector(p_in), q_out_, dt);
  q_out = q_out_[0];
  return out;
}

int MyIK::CartToJnt(const std::vector<KDL::JntArray>& q_init, const std::vector<KDL::Frame>& p_in, std::vector<KDL::JntArray>& q_out, const double& dt) {
  q_out = q_init;

  if (!initialized) {
    ROS_ERROR("IK was not properly initialized with a valid chain or limits. IK cannot proceed");
    return -1;
  }

  std::vector<std::vector<double>> artificial_lower_limits(nRobot), artificial_upper_limits(nRobot);
  std::vector<Affine3d> Ts_d(nRobot), Ts(nRobot);
  std::vector<KDL::JntArray> q(nRobot);

  for (int i = 0; i < nRobot; i++) {
    tf::transformKDLToEigen(p_in[i], Ts_d[i]);

    q[i].resize(myIKs[i]->getNJnt());
    q[i].data = myIKs[i]->getUpdatedJntLimit(q_init[i], artificial_lower_limits[i], artificial_upper_limits[i], dt);
  }

  double alpha = 1.;

  VectorXd w = VectorXd::Ones(nJnt) * 1.0e-3;
  for (int i = 1; i < nJnt; i++)
    w(i) = w(i - 1) * 3.0;
  MatrixXd W = w.asDiagonal();

  double w_n = 1.0e-5;

  // boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;

  ros::Time start_time_ros = ros::Time::now();
  while (ros::ok()) {
    int finished = 0;

    // diff = (boost::posix_time::microsec_clock::local_time() - start_time).total_nanoseconds() * 1.0e-9;
    double diff = (ros::Time::now() - start_time_ros).toSec();
    double time_left = dt - diff;
    if (time_left < 0.)
      break;

    for (int i = 0; i < nRobot; i++) {
      KDL::Frame p;
      KDL::Jacobian jac(myIKs[i]->getNJnt());
      JntToJac(q[i], jac);
      JntToCart(q[i], p);

      Affine3d T_d, T;
      T_d = Ts_d[i];
      tf::transformKDLToEigen(p, T);

      MatrixXd J = jac.data;
      VectorXd e = getCartError(T, T_d);
      // std::cout << e.transpose() << std::endl;

      if (e.dot(e) < eps) {
        finished += 1;
        continue;
      }

      // Compute pseudo-inverse Jacobian
      MatrixXd J_pinv, J_w_pinv;
      double gamma = 0.5 * e.transpose() * W * e + w_n;                                                                            // proposed by Sugihara-sensei
      J_w_pinv = (J.transpose() * W * J + gamma * MatrixXd::Identity(nJnt, nJnt)).colPivHouseholderQr().solve(J.transpose() * W);  // weighted & Levenberg–Marquardt

      // Update
      q[i].data = q[i].data + alpha * J_w_pinv * e;

      // check joint limit and nan error
      for (int j = 0; j < q[i].data.size(); j++) {
        if (std::isnan(q[i].data[j]))
          return -1;

        q[i].data[j] = std::min(std::max(q[i].data[j], artificial_lower_limits[i][j]), artificial_upper_limits[i][j]);
      }
    }

    if (finished == nRobot)
      break;
  }

  for (int i = 0; i < nRobot; i++)
    q_out[i].data = q[i].data;

  return 1;
}

void MyIK::updateVelP(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel, const VectorXd& e) {
  Matrix<double, 6, 1> v;
  tf::twistKDLToEigen(des_eff_vel, v);

  // std::cout << v.transpose() << std::endl;
  VectorXd kp = 3.0 * VectorXd::Ones(6);  // TODO: make this p gain as ros param
  kp.tail(3) = kp.tail(3) * 0.5 / M_PI * 0.5;
  v = v + kp.asDiagonal() * e;

  // std::cout << v.transpose() << std::endl;

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
#if 0
int MyIK::CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Twist& des_eff_vel, const VectorXd& e, KDL::JntArray& dq_des, const double& dt) {
  KDL::Jacobian jac(nJnt);
  JntToJac(q_cur, jac);
  MatrixXd J = jac.data;

  // KDL::Frame p;
  // JntToCart(q_cur, p);
  // Vector3d p_end;
  // tf::vectorKDLToEigen(p.p, p_end);

  Matrix<double, 6, 1> v;
  tf::twistKDLToEigen(des_eff_vel, v);

  VectorXd w = VectorXd::Ones(6);
  w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;

  double w_n = 1.0e-8;  // this leads dq -> 0 witch is conflict with additonal task
  double gamma = 0.5 * e.transpose() * w.asDiagonal() * e + w_n;

  VectorXd dV = VectorXd::Zero(nJnt);

  // dV(0) = -(q_cur.data(0) - 0.);
  // static ros::Time t0 = ros::Time::now();
  // double t = (ros::Time::now() - t0).toSec();
  // dV(0) = -(q_cur.data(0) - (0.3 * sin(2.0 * M_PI * 0.1 * t)));

  MatrixXd S = MatrixXd::Zero(1, nJnt);  // selection matrix
  S(0) = 1.0;

  std::vector<double> w_h = { 1.0e5, 0.0 };  //{ 1.0e5, 1.0e2 };
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

  int nCA = 0;
  std::vector<MatrixXd> A_ca;
  if (enableSelfCollisionAvoidance)
    nCA = addSelfCollisionAvoidance(q_cur, lower_vel_limits, upper_vel_limits, A_ca);

  Map<VectorXd> lowerBound(&lower_vel_limits[0], lower_vel_limits.size());
  Map<VectorXd> upperBound(&upper_vel_limits[0], upper_vel_limits.size());
  // SparseMatrix<double> linearMatrix(nJnt, nJnt);  // TODO: is it better to separate velocity and joint limit?
  // linearMatrix.setIdentity();

  MatrixXd A(nJnt + nCA, nJnt);
  A.block(0, 0, nJnt, nJnt) = MatrixXd::Identity(nJnt, nJnt);
  for (int i = 0; i < nCA; i++)
    A.block(nJnt + i, 0, 1, nJnt) = A_ca[i];
  SparseMatrix<double> linearMatrix = A.sparseView();

  // TODO: update variables. this seems to be difficult with OSQP since this library is for sparse QP optimization. When updating hessian matrix and changing its sparse form, the
  // function init the solver instanse. At that time, bounds is removed and failed to pass the bounds check. Other QP solver for dense problem would be a solusion. Actually, this
  // propblem matrix is not sparse.
  OsqpEigen::Solver qpSolver;
  qpSolver.settings()->setVerbosity(false);
  qpSolver.settings()->setWarmStart(true);
  // qpSolver.settings()->setAbsoluteTolerance(1.0e-6);
  // qpSolver.settings()->setRelativeTolerance(1.0e-6);

  // set the initial data of the QP solver
  qpSolver.data()->setNumberOfVariables(nJnt);
  qpSolver.data()->setNumberOfConstraints(nJnt + nCA);

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
#endif
int MyIK::CartToJntVel_qp(const KDL::JntArray& q_cur, const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel, KDL::JntArray& dq_des, const double& dt) {
  std::vector<KDL::JntArray> dq_des_ = std_utility::makeVector(dq_des);
  int out = this->CartToJntVel_qp(std_utility::makeVector(q_cur), std_utility::makeVector(des_eff_pose), std_utility::makeVector(des_eff_vel), dq_des_, dt);
  dq_des = dq_des_[0];
  return out;
}

int MyIK::CartToJntVel_qp(const std::vector<KDL::JntArray>& q_cur, const std::vector<KDL::Frame>& des_eff_pose, const std::vector<KDL::Twist>& des_eff_vel,
                          std::vector<KDL::JntArray>& dq_des, const double& dt) {
  if (!initialized) {
    ROS_ERROR("IK was not properly initialized with a valid chain or limits. IK cannot proceed");
    return -1;
  }

  std::vector<MatrixXd> Js(nRobot);
  std::vector<VectorXd> es(nRobot);
  std::vector<Matrix<double, 6, 1>> vs(nRobot);

  VectorXd kp = 3.0 * VectorXd::Ones(6);  // TODO: make this p gain as ros param
  kp.tail(3) = kp.tail(3) * 0.5 / M_PI * 0.5;

  for (int i = 0; i < nRobot; i++) {
    KDL::Jacobian jac(myIKs[i]->getNJnt());

    myIKs[i]->JntToJac(q_cur[i], jac);
    Js[i] = jac.data;

    KDL::Frame p;
    myIKs[i]->JntToCart(q_cur[i], p);

    Affine3d Ts_d, Ts;
    tf::transformKDLToEigen(des_eff_pose[i], Ts_d);
    tf::transformKDLToEigen(p, Ts);
    es[i] = getCartError(Ts, Ts_d);

    tf::twistKDLToEigen(des_eff_vel[i], vs[i]);

    if (!myIKs[i]->getPoseFeedbackDisabled()) {
      vs[i] = vs[i] + kp.asDiagonal() * es[i];
    }
  }

  std::vector<MatrixXd> Js_(nRobot);  // augumented Jacobian matrices
  for (int i = 0; i < nRobot; i++) {
    Js_[i] = MatrixXd::Zero(Js[i].rows(), nState);
    Js_[i].block(0, iJnt[i], Js[i].rows(), Js[i].cols()) = Js[i];
  }

  std::vector<MatrixXd> H(nRobot + nAddObj);
  std::vector<VectorXd> g(nRobot + nAddObj);

  // additonal objective term in QP (1) for singularity avoidance
  w_h[nRobot] = 1.0e-0;
  H[nRobot] = MatrixXd::Identity(nState, nState);
  g[nRobot] = VectorXd::Zero(nState);  // this will be updated in the loop below

  // additonal objective term in QP (2) for null space configuration
  w_h[nRobot + 1] = 1.0e-1;
  H[nRobot + 1] = MatrixXd::Identity(nState, nState);
  g[nRobot + 1] = (std_utility::concatenateVectors(q_rest) - std_utility::concatenateVectors(q_cur)).transpose();

  for (int i = 0; i < nRobot; i++) {
    VectorXd w = (VectorXd(6) << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI).finished() * 1.0e2;
    double w_n = 0.0;  // 1.0e-6;  // this leads dq -> 0 witch is conflict with additonal task
    double gamma = 0.5 * es[i].transpose() * w.asDiagonal() * es[i] + w_n;
    // std::cout << gamma << std::endl;
    H[i] = Js_[i].transpose() * Js_[i];
    g[i] = vs[i].transpose() * Js_[i];
    H[nRobot].block(iJnt[i], iJnt[i], myIKs[i]->getNJnt(), myIKs[i]->getNJnt()) = gamma * MatrixXd::Identity(myIKs[i]->getNJnt(), myIKs[i]->getNJnt());
  }

  // allocate QP problem matrices and vectores
  SparseMatrix<double> hessian = std_utility::weightedSum(w_h, H).sparseView();
  VectorXd gradient = -std_utility::weightedSum(w_h, g);

  std::vector<std::vector<double>> lower_vel_limits(nRobot), upper_vel_limits(nRobot);
  for (int i = 0; i < nRobot; i++)
    myIKs[i]->getUpdatedJntVelLimit(q_cur[i], lower_vel_limits[i], upper_vel_limits[i], dt);

  std::vector<double> lower_vel_limits_, upper_vel_limits_;
  for (auto&& v : lower_vel_limits) {
    lower_vel_limits_.insert(lower_vel_limits_.end(), v.begin(), v.end());
  }
  for (auto&& v : upper_vel_limits) {
    upper_vel_limits_.insert(upper_vel_limits_.end(), v.begin(), v.end());
  }

  // Collisiion Avoidance

  int nCA = 0;
  std::vector<MatrixXd> A_ca;

  // for (int i = 0; i < nRobot; i++)
  // if (enableSelfCollisionAvoidance)
  // nCA = addSelfCollisionAvoidance(q_cur[i], lower_vel_limits_, upper_vel_limits_, A_ca);

  if (enableCollisionAvoidance && nRobot > 1)
    nCA = addCollisionAvoidance(q_cur, lower_vel_limits_, upper_vel_limits_, A_ca);
  // nCA = addCollisionAvoidance(Ts, Js_, lower_vel_limits_, upper_vel_limits_, A_ca);

  Map<VectorXd> lowerBound(&lower_vel_limits_[0], lower_vel_limits_.size());
  Map<VectorXd> upperBound(&upper_vel_limits_[0], upper_vel_limits_.size());
  MatrixXd A(nState + nCA, nState);
  A.block(0, 0, nState, nState) = MatrixXd::Identity(nState, nState);
  for (int i = 0; i < nCA; i++)
    A.block(nState + i, 0, 1, nState) = A_ca[i];
  SparseMatrix<double> linearMatrix = A.sparseView();

  // TODO: update variables. this seems to be difficult with OSQP since this library is for sparse QP optimization. When updating hessian matrix and changing its sparse form, the
  // function init the solver instanse. At that time, bounds is removed and failed to pass the bounds check. Other QP solver for dense problem would be a solusion. Actually, this
  // propblem matrix is not sparse.
  OsqpEigen::Solver qpSolver;
  qpSolver.settings()->setVerbosity(false);
  qpSolver.settings()->setWarmStart(true);

  // qpSolver.settings()->setAbsoluteTolerance(1.0e-6);
  // qpSolver.settings()->setRelativeTolerance(1.0e-6);

  // set the initial data of the QP solver
  qpSolver.data()->setNumberOfVariables(nState);
  qpSolver.data()->setNumberOfConstraints(nState + nCA);

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

  // set the initial guess
  // if(primalVariable.size() == nState)
  // qpSolver.setPrimalVariable(primalVariable);  // TODO: verify if this is useful for fast optimization

  // solve the QP problem
  OsqpEigen::ErrorExitFlag a = qpSolver.solveProblem();
  if (a != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << (int)a << "\n";
    return -1;
  }

  // get the controller input
  VectorXd dq_des_ = qpSolver.getSolution();

  // check if the solution does not include nan
  if (dq_des_.hasNaN())
    return -1;

  for (int i = 0; i < nRobot; i++) {
    dq_des[i].resize(myIKs[i]->getNJnt());
    dq_des[i].data = dq_des_.segment(iJnt[i], myIKs[i]->getNJnt());
  }

  // qpSolver.getPrimalVariable(primalVariable);

  return 1;
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

int MyIK::addSelfCollisionAvoidance(const KDL::JntArray& q_cur, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca) {
  std::vector<KDL::Frame> frame(nSeg);
  fksolver->JntToCart(q_cur, frame);

  // positon at origin [0] + joints [1 ~ nJnt] +  eef [nJnt+1] (size nJnt+2)
  std::vector<Vector3d> p(nJnt + 2);
  tf::vectorKDLToEigen(frame[0].p, p[0]);
  for (int i = 0; i < nJnt; i++)
    tf::vectorKDLToEigen(frame[idxSegJnt[i]].p, p[i + 1]);
  tf::vectorKDLToEigen(frame.back().p, p[nJnt + 1]);

  // Jacobian at origin [0] + joints [1 ~ nJnt] +  eef [nJnt+1] (size nJnt+2)
  std::vector<KDL::Jacobian> J(nJnt + 2, KDL::Jacobian(nJnt));
  jacsolver->JntToJac(q_cur, J[0], 0);
  for (int i = 0; i < nJnt; i++)
    jacsolver->JntToJac(q_cur, J[i + 1], idxSegJnt[i]);
  JntToJac(q_cur, J[nJnt + 1]);

  double di = 0.10, ds = 0.08, eta = 0.1;  // TODO: make these parameters rosparam

  Vector3d p_end, p_min;
  MatrixXd J_end;
  int count = 0;
  for (int j = nJnt + 1; j > 2; j--) {  // set target end position and jacobian
    p_end = p[j];
    J_end = J[j].data;
    for (int i = 0; i < j - 2; i++) {  // search colliding point
      Vector3d rp = p[i + 1] - p[i];
      double rp_norm = rp.norm();
      double s = (rp / rp_norm).dot(p_end - p[i]);

      if (s < 0)
        p_min = p[i];
      else if (s > rp_norm)
        p_min = p[i + 1];
      else
        p_min = s * (rp / rp_norm);

      Vector3d d_vec = p_end - p_min;
      double d = d_vec.norm();

      if (d < di) {  // if the relative distance is smaller than influenced distance
        ROS_INFO_STREAM("Collision Detected >> #" << i << " and #" << j);
        A_ca.push_back((d_vec / d).transpose() * (J_end.block(0, 0, 3, nJnt) - J[i].data.block(0, 0, 3, nJnt)));
        lower_vel_limits_.push_back(-eta * (d - ds) / (di - ds));
        upper_vel_limits_.push_back(OsqpEigen::INFTY);

        count++;
      }
    }
  }

  return count;
}

void MyIK::setqRest(const std::vector<KDL::JntArray>& q_rest) {
  this->q_rest = q_rest;
}

int MyIK::addCollisionAvoidance(const std::vector<Affine3d>& Ts, const std::vector<MatrixXd>& Js_, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_,
                                std::vector<MatrixXd>& A_ca) {
  if (nRobot < 2)
    return 0;

  double ds = 0.10;
  double di = 0.15;
  double eta = 0.1;

  for (auto& comb : combsRobot) {
    // std::cout << comb[0] << "," << comb[1] << std::endl;
    // Ts[]
    Vector3d d_vec = (myIKs[comb[0]]->getT_base_world() * Ts[comb[0]]).translation() - (myIKs[comb[1]]->getT_base_world() * Ts[comb[1]]).translation();

    // std::cout << (myIKs[comb[0]]->getT_base_world() * Ts[comb[0]]).translation().transpose() << std::endl;
    // std::cout << (myIKs[comb[1]]->getT_base_world() * Ts[comb[1]]).translation().transpose() << std::endl;
    double d = d_vec.norm();
    // std::cout << d << std::endl;
    if (d < di) {
      A_ca.push_back((d_vec / d).transpose() * (myIKs[comb[0]]->getT_base_world().rotation() * Js_[comb[0]].block(0, 0, 3, nState) -
                                                myIKs[comb[1]]->getT_base_world().rotation() * Js_[comb[1]].block(0, 0, 3, nState)));
      lower_vel_limits_.push_back(-eta * (d - ds) / (di - ds));
      upper_vel_limits_.push_back(OsqpEigen::INFTY);
    }
  }
  return A_ca.size();
}

int MyIK::calcCollisionAvoidance(int c0, int c1, const std::vector<std::vector<Vector3d>>& p_all, const std::vector<std::vector<KDL::Jacobian>>& J_all, double ds, double di,
                                 double eta, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca) {
  int nCollision = 0;
  std::vector<Vector3d> p0 = p_all[c0], p1 = p_all[c1];
  std::vector<KDL::Jacobian> J0 = J_all[c0], J1 = J_all[c1];
  int iJnt0 = iJnt[c0], iJnt1 = iJnt[c1];
  Matrix3d R0 = myIKs[c0]->getT_base_world().rotation(), R1 = myIKs[c1]->getT_base_world().rotation();

  // std::cout << p0.size() << " " << p1.size() << std::endl;

  for (int i = 0; i < p0.size() - 1; i++) {
    int j_start = 0;
    if (c0 == c1)  // in case of self collision check
      j_start = i + 2;

    for (int j = j_start; j < p1.size() - 1; j++) {
      double as, bs;
      if (!getClosestPointLineSegments(p0[i], p0[i + 1], p1[j], p1[j + 1], as, bs))
        continue;
      Vector3d d_vec = getVec(p0[i], p0[i + 1], p1[j], p1[j + 1], as, bs);
      double d = d_vec.norm();

      // ROS_INFO_STREAM("d: " << d << " i:" << i << " j:" << j << " as: " << as << " bs: " << bs);

      if (d < di) {  // if the relative shortest distance is smaller than the infulenced distance
        // std::cout << "collision detected between " << i << " and " << j << std::endl;

        // get extended Jacobian
        MatrixXd Js_0 = MatrixXd::Zero(J0[i + 1].rows(), nState);
        Js_0.block(0, iJnt0, J0[i + 1].data.rows(), J0[i + 1].data.cols()) = J0[i + 1].data;

        MatrixXd Js_1 = MatrixXd::Zero(J1[j + 1].rows(), nState);
        Js_1.block(0, iJnt1, J1[j + 1].data.rows(), J1[j + 1].data.cols()) = J1[j + 1].data;

        // scale a col of Jacobian related to the closest point
        Js_0.block(0, iJnt0 + i - 1, 6, 1) *= as;
        Js_1.block(0, iJnt1 + j - 1, 6, 1) *= bs;

        // push back inequality constraint for collision avoidance
        A_ca.push_back((d_vec / d).transpose() * (R0 * Js_0.block(0, 0, 3, nState) - R1 * Js_1.block(0, 0, 3, nState)));
        lower_vel_limits_.push_back(-eta * (d - ds) / (di - ds));
        upper_vel_limits_.push_back(OsqpEigen::INFTY);

        nCollision++;
      }
    }
  }

  return nCollision;
}

int MyIK::addCollisionAvoidance(const std::vector<KDL::JntArray>& q_cur, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_,
                                std::vector<MatrixXd>& A_ca) {
  std::vector<std::vector<Vector3d>> p_all(nRobot);
  std::vector<std::vector<KDL::Jacobian>> J_all(nRobot);

  for (int i = 0; i < nRobot; i++) {
    std::vector<KDL::Frame> frame(myIKs[i]->getNSeg());
    myIKs[i]->JntToCart(q_cur[i], frame);

    int nJnt = myIKs[i]->getNJnt();
    std::vector<int> idxSegJnt = myIKs[i]->getIdxSegJnt();

    // positon at origin [0] + joints [1 ~ nJnt] +  eef [nJnt+1] (size nJnt+2)
    std::vector<Vector3d> p(nJnt + 2);
    tf::vectorKDLToEigen(frame[0].p, p[0]);
    for (int j = 0; j < nJnt; j++)
      tf::vectorKDLToEigen(frame[idxSegJnt[j]].p, p[j + 1]);
    tf::vectorKDLToEigen(frame.back().p, p[nJnt + 1]);
    p_all[i] = p;

    // Jacobian at origin [0] + joints [1 ~ nJnt] +  eef [nJnt+1] (size nJnt+2)
    std::vector<KDL::Jacobian> J(nJnt + 2, KDL::Jacobian(nJnt));
    myIKs[i]->JntToJac(q_cur[i], J[0], 0);
    for (int j = 0; j < nJnt; j++)
      myIKs[i]->JntToJac(q_cur[i], J[j + 1], idxSegJnt[j]);
    myIKs[i]->JntToJac(q_cur[i], J[nJnt + 1]);
    J_all[i] = J;
  }

  for (int i = 0; i < nRobot; i++)
    for (int j = 0; j < p_all[i].size(); j++)
      p_all[i][j] = (myIKs[i]->getT_base_world() * (Vector4d() << p_all[i][j], 1.0).finished()).head(3);

  double ds = 0.10;
  double di = 0.20;
  double eta = 0.2;

  int nCollision = 0;

  // self collision avoidance
  // for (int i = 0; i < nRobot; i++)
  // nCollision += calcCollisionAvoidance(i, i, p_all, J_all, 0.1, 0.15, eta, lower_vel_limits_, upper_vel_limits_, A_ca);

  // collision avoidance against other robots
  if (nRobot > 1) {
    for (auto& comb : combsRobot)
      nCollision += calcCollisionAvoidance(comb[0], comb[1], p_all, J_all, ds, di, eta, lower_vel_limits_, upper_vel_limits_, A_ca);
  }

  // ROS_INFO_STREAM("nCollision: " << nCollision);

  return A_ca.size();
}

// get closest distance and point between two line segments
bool MyIK::getClosestPointLineSegments(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, double& as, double& bs) {
  Vector3d a = a1 - a0;
  Vector3d b = b1 - b0;
  double a_norm = a.norm();
  double b_norm = b.norm();

  if (a_norm < std::abs(1.0e-6) || b_norm < std::abs(1.0e-6) || (a1 - b0).norm() < std::abs(1.0e-6))  // zero length case
    return false;

  // std::cout << a_norm << " " << b_norm << std::endl;

  Vector3d na = a / a_norm;
  Vector3d nb = b / b_norm;

  Vector3d cross_na_nb = na.cross(nb);
  double denom = cross_na_nb.squaredNorm();

  if (denom < 1.0e-6) {  // parallel case
    double d0 = na.dot(b0 - a0);
    double d1 = nb.dot(b1 - a0);

    if (d0 < 0.0 && d1 < 0.0) {
      if (std::abs(d0) < std::abs(d1)) {
        as = 0.0;
        bs = 0.0;
        return true;
      } else if (d0 > a_norm && d1 > a_norm) {
        if (std::abs(d0) < std::abs(d1)) {
          as = 1.0;
          bs = 0.0;
          return true;
        } else {
          as = 1.0;
          bs = 1.0;
          return true;
        }
      }
    }
    // overlap case
    as = 0.0;
    bs = 0.0;
    return true;

  } else {
    // lines criss cross case
    Vector3d t = b0 - a0;
    as = ((Matrix3d() << t, nb, cross_na_nb).finished().determinant() / denom) / a_norm;
    bs = ((Matrix3d() << t, na, cross_na_nb).finished().determinant() / denom) / b_norm;

    if (as < 0.0 || as > 1.0) {
      if (as < 0.0) {
        as = 0.0;
      } else if (as > 1.0) {
        as = 1.0;
      }

      Vector3d pA = a0 + as * a;
      double dot = nb.dot(pA - b0);
      if (dot < 0)
        bs = 0.0;
      else if (dot > b_norm)
        bs = 1.0;
      else
        bs = dot / b_norm;
    }

    if (bs < 0.0 || bs > 1.0) {
      if (bs < 0.0) {
        bs = 0.0;
      } else if (bs > 1.0) {
        bs = 1.0;
      }

      Vector3d pB = b0 + bs * b;
      double dot = na.dot(pB - a0);
      if (dot < 0)
        as = 0.0;
      else if (dot > a_norm)
        as = 1.0;
      else
        as = dot / a_norm;
    }
  }
  return true;
}

// get distance
double MyIK::getDistance(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, const double& as, const double& bs) {
  return getVec(a0, a1, b0, b1, as, bs).norm();
}

Vector3d MyIK::getVec(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, const double& as, const double& bs) {
  Vector3d a = a1 - a0;
  Vector3d b = b1 - b0;
  return a0 + as * a - (b0 + bs * b);
}

void MyIK::resetRobotWeight() {
  w_h = init_w_h;
}
void MyIK::setRobotWeight(int robotIndex, double rate) {
  w_h[robotIndex] = rate * init_w_h[robotIndex];
}
};  // namespace MyIK
