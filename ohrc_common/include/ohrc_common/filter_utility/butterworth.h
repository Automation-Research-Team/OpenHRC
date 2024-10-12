#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

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
  Eigen::Vector3d vx, vy;

  // filter coefficients
  Eigen::Vector3d a, b;

  bool firsttime = false;
};

#endif  // BUTTERWORTH_H
