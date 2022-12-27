#include "ohrc_control/multi_test_controller.hpp"

MultiTestController::MultiTestController() {
}

void MultiTestController::runLoopEnd() {
  // TODO: Save solusion accuracy and calcuration time
}

void MultiTestController::updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  static ros::Time t0 = ros::Time::now();
  tf::transformEigenToKDL(controller->getT_init(), pose);

  int axis = 1;
  double A = 0.2, f = 0.1;

  double t = (ros::Time::now() - t0).toSec();
  pose.p.data[axis] = A * sin(2. * M_PI * f * t) + pose.p.data[axis];
  twist.vel.data[axis] = A * 2. * M_PI * f * cos(2. * M_PI * f * t);
  controller->enableOperation();
}

// std::vector<Vector3d> projection(std::vector<Vector3d> X, std::vector<int> n) {
//   Vector3d p = X[n[2]] - X[n[0]];
//   Vector3d nz = (X[n[2]] - X[n[0]]);
//   nz /= nz.norm();
//   Vector3d nx = (p - p.transpose() * nz);
//   nx /= nz.norm();
//   Vector3d ny = nz.cross(nx);
//   Matrix3d R;
//   R << nx, ny, nz;

//   std::vector<Vector3d> X_(X.size());
//   for (int i = 0; i < X.size(); i++) {
//     X_[i] = R.inverse() * (X[i] - X[n[0]]);
//   }
//   return X_;
// }

// void locallyLinearMapping(Vector3d x, Vector3d dx, std::vector<Vector3d> X, std::vector<Vector3d> Y, Vector3d y, Vector3d dy) {
//   int N = X.size();
//   std::vector<int> n(3);
//   std::vector<double> distances(N);
//   for (auto& dist : distances)
//     dist = (X[i] - x).norm();
//   n[0] = std_utility::max_index(distances);

//   for (n[1] = 1; i < N; n[1]++) {
//     if (n[1] == n[0])
//       continue;

//     X.push_back(x);
//     std::vector<Vector3d> X_ = projection(X, n);
//     std::vector<Vector3d> Y_ = projection(Y, n);
//   }
// }

void MultiTestController::updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  if (MFmode == MFMode::Individual) {
    static ros::Time t0 = ros::Time::now();
    tf::transformEigenToKDL(controller->getT_init(), pose);

    int axis = 0;
    double A = 0.25, f = 0.1;

    double t = (ros::Time::now() - t0).toSec();
    pose.p.data[axis] = -A * sin(2. * M_PI * f * t) + pose.p.data[axis];
    twist.vel.data[axis] = -A * 2. * M_PI * f * cos(2. * M_PI * f * t);
  } else if (MFmode == MFMode::Parallel) {
    Vector3d x, dx, y, dy;
    std::vector<Vector3d> X, Y;
    // locallyLinearMapping(x, dx, X, Y, y, dy)
  }

  controller->enableOperation();
}