#ifndef KALMAN_H
#define KALMAN_H

#include <ros/ros.h>

#include <Eigen/Eigen>

using namespace Eigen;

class Kalman {
  MatrixXd F, H, R, Q, P, I;
  VectorXd x;
  unsigned int dim, z_dim;
  bool modelInitialized = false;

 public:
  bool first_prediction = true;

  Kalman();
  Kalman(unsigned int dim);

  void init_model();
  void init_model(const VectorXd q, const VectorXd r);

  void init(const VectorXd x);
  void init(const VectorXd x, const MatrixXd P);
  void init(const VectorXd x, const VectorXd q, const VectorXd r);

  void init(const VectorXd x, const MatrixXd P, MatrixXd F, MatrixXd H, MatrixXd Q, MatrixXd R);
  VectorXd predict();

  VectorXd update(const VectorXd z);
  double likelihood(VectorXd p);

  double likelihood_z(VectorXd p);
  double gaussian(int n, VectorXd x, VectorXd mu, MatrixXd Sigma);

  double likelihood_nSigma(double n);

  double likelihood_z_nSigma(double n);

  VectorXd getnSigma(double n);

  VectorXd getnSigma_z(double n);
  MatrixXd getH();
};

#endif  // KALMAN_H
