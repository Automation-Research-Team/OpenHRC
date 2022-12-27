#ifndef BAYES_NORMAL_GAMMA_H
#define BAYES_NORMAL_GAMMA_H

#include <Eigen/Eigen>

class BayesNormalGamma {
  double mu0, zeta0, alpha0, beta0;

 public:
  BayesNormalGamma(double mu0);
  BayesNormalGamma(double mu0, double zeta0, double alpha0, double beta0);

  double estimate(Eigen::ArrayXd value);
};

#endif  // BAYES_NORMAL_GAMMA_H