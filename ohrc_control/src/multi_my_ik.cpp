#include "ohrc_control/multi_my_ik.hpp"

namespace MyIK {
MultiMyIK::MultiMyIK(const std::vector<std::string>& base_link, const std::vector<std::string>& tip_link, const std::vector<std::string>& URDF_param,
                     const std::vector<Affine3d>& T_base_world, const std::vector<std::shared_ptr<MyIK>>& myik_ptr, double _eps, SolveType _type)
  : nRobot(base_link.size()), initialized(false), eps(_eps), solvetype(_type) {
  myIKs.resize(nRobot);
  // for (int i = 0; i < nRobot; i++)
  // myIKs[i].reset(new MyIK(base_link[i], tip_link[i], URDF_param[i], _eps, T_base_world[i], _type));
  myIKs = myik_ptr;

  iJnt.resize(nRobot, 0);
  for (int i = 0; i < nRobot; i++) {
    nState += myIKs[i]->getNJnt();

    if (i > 0)
      iJnt[i] = iJnt[i - 1] + myIKs[i]->getNJnt();  // all_cols;
  }

  if (nRobot > 1)
    combsRobot = std_utility::comb(nRobot, 2);
  // combsLink = std_utility::comb(nJnt, 2);
  init_w_h.resize(nRobot + 1, 1.0e5);
  w_h = init_w_h;
}

int MultiMyIK::CartToJnt(const std::vector<KDL::JntArray>& q_init, const std::vector<KDL::Frame>& p_in, std::vector<KDL::JntArray>& q_out, const double& dt) {
  q_out = q_init;

  // if (!initialized) {
  //   ROS_ERROR("TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
  //   return -1;
  // }

  // if (q_init.data.size() != types.size()) {
  //   ROS_ERROR_THROTTLE(1.0, "IK seeded with wrong number of joints.  Expected %d but got %d", (int)types.size(), (int)q_init.data.size());
  //   return -3;
  // }

  std::vector<std::vector<double>> artificial_lower_limits(nRobot), artificial_upper_limits(nRobot);
  std::vector<VectorXd> xs(nRobot);
  for (int i = 0; i < nRobot; i++)
    xs[i] = myIKs[i]->getUpdatedJntLimit(q_init[i], artificial_lower_limits[i], artificial_upper_limits[i], dt);

  std::vector<Affine3d> Ts_d(nRobot), Ts(nRobot);

  for (int i = 0; i < nRobot; i++) {
    tf::transformKDLToEigen(p_in[i], Ts_d[i]);
  }

  double alpha = 0.01;
  double w_n = 1.0e-3;

  VectorXd x = VectorXd::Zero(nState);

  boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;
  while (ros::ok()) {
    diff = boost::posix_time::microsec_clock::local_time() - start_time;

    MatrixXd J_w_pinv = MatrixXd::Zero(6 * nRobot, nState);
    VectorXd e = VectorXd::Zero(6 * nRobot);
    int finished = 0;
    for (int i = 0; i < nRobot; i++) {
      KDL::Jacobian jac(myIKs[i]->getNJnt());
      KDL::JntArray q(myIKs[i]->getNJnt());

      q.data = xs[i];
      myIKs[i]->JntToJac(q, jac);

      KDL::Frame p;
      myIKs[i]->JntToCart(q, p);
      tf::transformKDLToEigen(p, Ts[i]);

      VectorXd e_ = getCartError(Ts[i], Ts_d[i]);

      MatrixXd J_pinv, J_w_pinv_;

      VectorXd w(6);
      w << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI;
      MatrixXd W = w.asDiagonal();

      // VectorXd w = VectorXd::Ones(myIKs[i]->getNJnt());
      // for (int i = 1; i < myIKs[i]->getNJnt(); i++)
      //   w(i) = w(i - 1) * 3.0;
      // MatrixXd W = w.asDiagonal();

      double gamma = 0.5 * e_.transpose() * W * e_ + w_n;
      MatrixXd J = jac.data;                                                                                                                     // proposed by Sugihara-sensei
      J_w_pinv_ = (J.transpose() * W * J + gamma * MatrixXd::Identity(myIKs[i]->getNJnt(), myIKs[i]->getNJnt())).inverse() * J.transpose() * W;  // weighted & Levenberg–Marquardt/
      J_w_pinv.block(6 * i, iJnt[i], J_w_pinv_.rows(), J_w_pinv_.cols()) = J_w_pinv_;

      e.segment(i * 6, 6) = e_;
      x.segment(iJnt[i], myIKs[i]->getNJnt()) = xs[i];

      if (e.dot(e) < eps)
        finished += 1;

      // ROS_INFO_STREAM(e.dot(e));
    }
    double time_left = dt - diff.total_nanoseconds() * 1.0e-9;
    if (time_left < 0.0)
      return -1;

    if (finished == nRobot)
      break;

    x = x + alpha * J_w_pinv * e;

    for (int i = 0; i < nRobot; i++) {
      xs[i] = x.segment(iJnt[i], myIKs[i]->getNJnt());
      for (int j = 0; j < xs[i].size(); j++) {
        if (std::isnan(xs[i][j]))
          return -1;

        xs[i][j] = std::min(std::max(xs[i][j], artificial_lower_limits[i][j]), artificial_upper_limits[i][j]);
      }
    }
  }

  for (int i = 0; i < nRobot; i++)
    q_out[i].data = xs[i];

  return 1;
}

int MultiMyIK::CartToJntVel_qp(const std::vector<KDL::JntArray>& q_cur, const std::vector<KDL::Frame>& des_eff_pose, const std::vector<KDL::Twist>& des_eff_vel,
                               std::vector<KDL::JntArray>& dq_des, const double& dt) {
  std::vector<MatrixXd> Js(nRobot);
  std::vector<Affine3d> Ts_d(nRobot), Ts(nRobot);
  std::vector<VectorXd> es(nRobot);
  std::vector<Matrix<double, 6, 1>> vs(nRobot);

  for (int i = 0; i < nRobot; i++) {
    KDL::Jacobian jac(myIKs[i]->getNJnt());
    myIKs[i]->JntToJac(q_cur[i], jac);
    Js[i] = jac.data;

    KDL::Frame p;
    myIKs[i]->JntToCart(q_cur[i], p);

    tf::transformKDLToEigen(des_eff_pose[i], Ts_d[i]);
    tf::transformKDLToEigen(p, Ts[i]);
    es[i] = getCartError(Ts[i], Ts_d[i]);

    tf::twistKDLToEigen(des_eff_vel[i], vs[i]);
    VectorXd kp = 2.0 * VectorXd::Ones(6);  // TODO: make this p gain as ros param
    // kp.tail(3) = kp.tail(3) * 0.5 / M_PI;
    vs[i] = vs[i] + kp.asDiagonal() * es[i];
  }

  std::vector<MatrixXd> Js_(nRobot);
  for (int i = 0; i < nRobot; i++) {
    Js_[i] = MatrixXd::Zero(Js[i].rows(), nState);
    Js_[i].block(0, iJnt[i], Js[i].rows(), Js[i].cols()) = Js[i];
  }

  // w_h[0] = 1.0e3;
  w_h[nRobot] = 1.0e6;
  std::vector<MatrixXd> H(w_h.size());
  std::vector<VectorXd> g(w_h.size());
  H[w_h.size() - 1] = MatrixXd::Identity(nState, nState);
  g[w_h.size() - 1] = VectorXd::Zero(nState);

  for (int i = 0; i < nRobot; i++) {
    VectorXd w = (VectorXd(6) << 1.0, 1.0, 1.0, 0.5 / M_PI, 0.5 / M_PI, 0.5 / M_PI).finished();
    w *= 1.0;
    double w_n = 1.0e-8;  // this leads dq -> 0 witch is conflict with additonal task
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
  if (enableCollisionAvoidance)
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

  // solve the QP problem
  OsqpEigen::ErrorExitFlag a = qpSolver.solveProblem();
  if (a != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << (int)a << "\n";
    return -1;
  }

  // get the controller input
  VectorXd dq_des_ = qpSolver.getSolution();

  for (int i = 0; i < nRobot; i++) {
    dq_des[i].resize(myIKs[i]->getNJnt());
    dq_des[i].data = dq_des_.segment(iJnt[i], myIKs[i]->getNJnt());
  }

  return 1;
}

int MultiMyIK::addCollisionAvoidance(const std::vector<Affine3d>& Ts, const std::vector<MatrixXd>& Js_, std::vector<double>& lower_vel_limits_,
                                     std::vector<double>& upper_vel_limits_, std::vector<MatrixXd>& A_ca) {
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

int MultiMyIK::addCollisionAvoidance(const std::vector<KDL::JntArray>& q_cur, std::vector<double>& lower_vel_limits_, std::vector<double>& upper_vel_limits_,
                                     std::vector<MatrixXd>& A_ca) {
  if (nRobot < 2)
    return 0;

  std::vector<std::vector<Vector3d>> p_all(nRobot);
  std::vector<std::vector<KDL::Jacobian>> J_all(nRobot);

  for (int i = 0; i < nRobot; i++) {
    KDL::Chain chain;
    myIKs[i]->getKDLChain(chain);
    std::vector<KDL::Frame> frame(chain.getNrOfSegments());
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

  for (auto& comb : combsRobot) {
    std::vector<Vector3d> p0 = p_all[comb[0]];
    std::vector<Vector3d> p1 = p_all[comb[1]];

    double ds = 0.10;
    double di = 0.15;
    double eta = 0.1;

    for (int i = 0; i < p0.size() - 1; i++) {
      for (int j = 0; j < p1.size() - 1; j++) {
        double as, bs;
        getClosestPointLineSegments(p0[i], p0[i + 1], p1[j], p1[j + 1], as, bs);
        double d = getDistance(p0[i], p0[i + 1], p1[j], p1[j + 1], as, bs);

        // if (i == p0.size() - 2 && j == p0.size() - 3)
        // ROS_INFO_STREAM("d: " << d << " i:" << i << " j:" << j << " as: " << as << " bs: " << bs);
        // ROS_INFO_STREAM("p0:" << p0[i + 1].transpose() << " p1:" << p1[j + 1].transpose());
        if (d < di) {
          // ROS_INFO_STREAM("d: " << d << " i:" << i << " j:" << j << " as: " << as << " bs: " << bs);
          MatrixXd Js_0 = MatrixXd::Zero(J_all[comb[0]][i + 1].rows(), nState);
          Js_0.block(0, iJnt[comb[0]], J_all[comb[0]][i + 1].data.rows(), J_all[comb[0]][i + 1].data.cols()) = J_all[comb[0]][i + 1].data;

          MatrixXd Js_1 = MatrixXd::Zero(J_all[comb[1]][j + 1].rows(), nState);
          Js_1.block(0, iJnt[comb[1]], J_all[comb[1]][j + 1].data.rows(), J_all[comb[1]][j + 1].data.cols()) = J_all[comb[1]][j + 1].data;

          // ROS_INFO_STREAM(J_all[comb[0]][i + 1].data);
          // ROS_INFO_STREAM("0: " << Js_0);
          // ROS_INFO_STREAM(Js_0.block(0, iJnt[comb[0]] + i - 1, 6, 1));
          Js_0.block(0, iJnt[comb[0]] + i - 1, 6, 1) *= as;
          Js_1.block(0, iJnt[comb[1]] + j - 1, 6, 1) *= bs;

          Vector3d d_vec = p0[i] + as * (p0[i + 1] - p0[i]) - (p1[j] + bs * (p1[j + 1] - p1[j]));
          A_ca.push_back((d_vec / d).transpose() *
                         (myIKs[comb[0]]->getT_base_world().rotation() * Js_0.block(0, 0, 3, nState) - myIKs[comb[1]]->getT_base_world().rotation() * Js_1.block(0, 0, 3, nState)));
          lower_vel_limits_.push_back(-eta * (d - ds) / (di - ds));
          upper_vel_limits_.push_back(OsqpEigen::INFTY);
        }
      }
    }
  }
  return A_ca.size();
}

// get closest distance and point between two line segments
void MultiMyIK::getClosestPointLineSegments(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, double& as, double& bs) {
  Vector3d a = a1 - a0;
  Vector3d b = b1 - b0;
  double a_norm = a.norm();
  double b_norm = b.norm();

  Vector3d na = a / a_norm;
  Vector3d nb = b / b_norm;

  Vector3d cross_na_nb = na.cross(nb);
  double denom = cross_na_nb.squaredNorm();

  if (denom < 1.0e-6) {  // parallel case
    double d0 = na.dot(b0 - a0);
    double d1 = nb.dot(b1 - a0);

    if (d0 < 0.0 && d1 < 0.0) {
      if (abs(d0) < abs(d1)) {
        as = 0.0;
        bs = 0.0;
        return;
      } else if (d0 > a_norm && d1 > a_norm) {
        if (abs(d0) < abs(d1)) {
          as = 1.0;
          bs = 0.0;
          return;
        } else {
          as = 1.0;
          bs = 1.0;
          return;
        }
      }
    }
    // overlap case
    as = 0.0;
    bs = 0.0;
    return;

  } else {
    // lines criss cross case
    Vector3d t = b0 - a0;
    double ad = (Matrix3d() << t, nb, cross_na_nb).finished().determinant() / denom;
    double bd = (Matrix3d() << t, na, cross_na_nb).finished().determinant() / denom;

    as = ad / a_norm;
    bs = bd / b_norm;

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
  return;
}

// get distance
double MultiMyIK::getDistance(const Vector3d& a0, const Vector3d& a1, const Vector3d& b0, const Vector3d& b1, const double& as, const double& bs) {
  Vector3d a = a1 - a0;
  Vector3d b = b1 - b0;
  Vector3d d = a0 + as * a - (b0 + bs * b);
  return d.norm();
}

void MultiMyIK::resetRobotWeight() {
  w_h = init_w_h;
}
void MultiMyIK::setRobotWeight(int robotIndex, double rate) {
  w_h[robotIndex] = rate * init_w_h[robotIndex];
}
};  // namespace MyIK