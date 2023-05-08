#include "ohrc_common/filter_utility/butterworth.h"

butterworth::butterworth(int order, double cutoff_freq, double sample_freq) {
  if (order != 2) {
    ROS_ERROR_STREAM("butterworth: " << order << "-th order butterworth is still NOT implimented.");
    ros::shutdown();
  }

  if (cutoff_freq > 0.5 * sample_freq) {
    ROS_ERROR_STREAM("butterworth: cutoff frequency (" << cutoff_freq << "Hz) must be smaller than half of sampling rate (" << sample_freq << "Hz)!");
    ros::shutdown();
  }

  get_coefficients(cutoff_freq, sample_freq);

  vx = Eigen::Vector3d::Zero();
  vy = Eigen::Vector3d::Zero();
}

void butterworth::get_coefficients(double cutoff_freq, double sample_freq) {
  double T = 1.0 / sample_freq;

  double wc = 2.0 / T * tan(2.0 * M_PI * cutoff_freq * T / 2.0);  // cutoff prewarp frequency
  double den = (T * T) * (wc * wc) + sqrt(2.0) * T * wc * 2.0 + 4.0;

  b << (T * T) * (wc * wc) / den, (T * T) * (wc * wc) * 2.0 / den, (T * T) * (wc * wc) / den;

  a << 1.0, ((T * T) * (wc * wc) * 2.0 - 8.0) / den, ((T * T) * (wc * wc) - sqrt(2.0) * T * wc * 2.0 + 4.0) / den;
}

double butterworth::filter(double x) {
  if (!firsttime) {
    vx << x, x, x;
    vy << x, x, x;

    firsttime = true;
  } else {
    vx << x, vx(0), vx(1);
    vy << vy(0), vy(0), vy(1);
  }

  vy(0) = b(0) * vx(0) + b(1) * vx(1) + b(2) * vx(2) - a(1) * vy(1) - a(2) * vy(2);

  double y = vy(0);  // output

  if (fabs(y) < 1.0E-9)
    y = 0.0;

  return y;  // filter out
}
