#include "ohrc_common/filter_utility/bayes_normal_gamma.h"

BayesNormalGamma::BayesNormalGamma(double mu0) {
  BayesNormalGamma(mu0, 1., 1., 1. / mu0);
}
BayesNormalGamma::BayesNormalGamma(double mu0, double zeta0, double alpha0, double beta0) {
  this->mu0 = mu0;
  this->zeta0 = zeta0;
  this->alpha0 = alpha0;
  this->beta0 = beta0;
}

double BayesNormalGamma::estimate(Eigen::ArrayXd value) {
  int N = value.size();

  double muN = 1. / (N + this->beta0) * value.sum() + this->beta0 / (N + this->beta0) * this->mu0;
  double zetaN = N + zeta0;

  double alphaN = N * 0.5 + this->alpha0;
  double betaN = 0.5 * ((value * value).sum() + this->zeta0 * this->mu0 * this->mu0 - zetaN * muN * muN) * this->beta0;

  this->mu0 = muN;
  this->zeta0 = zetaN;
  this->alpha0 = alphaN;
  this->beta0 = betaN;

  return muN;
}