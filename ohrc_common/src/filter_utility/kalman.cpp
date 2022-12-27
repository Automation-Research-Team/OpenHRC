#include "ohrc_common/filter_utility/kalman.h"

Kalman::Kalman(){};

Kalman::Kalman(unsigned int dim) {  // for constant position model
  this->dim = dim;
  init_model();
}

void Kalman::init_model() {  // for constant position model
  init_model(VectorXd::Ones(dim) * 0.03 * 0.03, VectorXd::Ones(dim) * 0.02 * 0.02);
}

void Kalman::init_model(const VectorXd q, const VectorXd r) {  // for constant position model
  F = MatrixXd::Identity(dim, dim);
  H = MatrixXd::Identity(dim, dim);

  Q = q.asDiagonal();
  R = r.asDiagonal();

  x = VectorXd::Zero(dim);
  P = MatrixXd::Zero(dim, dim);

  I = MatrixXd::Identity(dim, dim);

  modelInitialized = true;
}

void Kalman::init(const VectorXd x) {  // for constant position model
  if (!modelInitialized) {
    this->dim = x.size();
    this->z_dim = x.size();
    init_model();
  }

  this->x = x;
  first_prediction = false;
}

void Kalman::init(const VectorXd x, const VectorXd q, const VectorXd r) {  // for constant position model
  if (!modelInitialized) {
    this->dim = x.size();
    this->z_dim = x.size();
    init_model(q, r);
  }

  this->x = x;
  first_prediction = false;
}

void Kalman::init(const VectorXd x, const MatrixXd P) {  // for constant position model
  this->init(x);
  this->P = P;
}

void Kalman::init(const VectorXd x, const MatrixXd P, MatrixXd F, MatrixXd H, MatrixXd Q, MatrixXd R) {
  this->x = x;
  this->P = P;

  this->F = F;
  this->H = H;
  this->Q = Q;
  this->R = R;

  this->dim = x.size();
  this->z_dim = (H * x).size();

  I = MatrixXd::Identity(dim, dim);

  first_prediction = false;
}

VectorXd Kalman::predict() {
  if (first_prediction) {
    ROS_ERROR_STREAM("Please initialize first");
    return VectorXd::Zero(dim);
  }

  x = F * x;
  P = F * P * F.transpose() + Q;
  return x;
}

VectorXd Kalman::update(const VectorXd z) {
  MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  x = x + K * (z - H * x);
  P = (I - K * H) * P;

  return x;
}

double Kalman::likelihood(VectorXd p) {
  // normal distribution
  return gaussian(dim, p, x, P);
}

double Kalman::likelihood_z(VectorXd p) {
  // normal distribution
  VectorXd z = H * x;
  MatrixXd Pz = H * P * H.transpose();

  return gaussian(z_dim, p, z, Pz);
}

double Kalman::gaussian(int n, VectorXd x, VectorXd mu, MatrixXd Sigma) {
  return 1.0 / (pow(sqrt(2.0 * M_PI), n) * sqrt(Sigma.determinant())) * exp(-0.5 * (x - mu).transpose() * Sigma.inverse() * (x - mu));
}

double Kalman::likelihood_nSigma(double n) {
  return likelihood(getnSigma(n));
}

double Kalman::likelihood_z_nSigma(double n) {
  return likelihood_z(getnSigma_z(n));
}

VectorXd Kalman::getnSigma(double n) {
  return (P.diagonal().array().sqrt() * n).matrix() + x;
}

VectorXd Kalman::getnSigma_z(double n) {
  return H * ((P.diagonal().array().sqrt() * n).matrix() + x);
}

MatrixXd Kalman::getH() {
  return H;
}