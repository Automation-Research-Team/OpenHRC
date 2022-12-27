#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#include <ros/ros.h>
#include <Eigen/Eigen>

class butterworth {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // constcuctor
  butterworth(int order, double cutoff_freq, double sample_freq);
  double filter(double x);

private:
  void get_coefficients(double cutoff_freq, double sample_freq);

  // internal filter variables
  Eigen::Vector3d vx; ///< contains x[k], x[k-1], x[k-2]
  Eigen::Vector3d vy; ///< contains y[k], y[k-1], y[k-2]

  // filter coefficients (b: num, a: den, same notation as in matlab)
  Eigen::Vector3d a; ///< filter coefficients: den
  Eigen::Vector3d b; ///< filter coefficients: num

  bool firsttime = false; ///< firsttime flag
};

#endif // BUTTERWORTH_H
