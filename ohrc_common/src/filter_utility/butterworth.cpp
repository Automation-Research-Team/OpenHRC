/*****************************************************************************/ /**
     @file butter2_class.cpp

     Class for 2nd order butterworth filter Butter2

     August 2015, Jun Nakanishi
 **********************************************************************************/
/**
 * @file butterworth.cpp
 * @author Shunki Itadera
 * @brief 2nd order butterworh low pass filter
 * @brief This class was written based on butter2_class.cpp (Jun Nakanishi, Aug. 2015)
 **/

#include "ohrc_common/filter_utility/butterworth.h"

/*********************************************************************************
    myButter2 constructor
**********************************************************************************/
butterworth::butterworth(int order, double cutoff_freq, double sample_freq) {
  /// Constructor
  /// @param[in] coff cutoff frequency (Hz)
  /// @param[in] sampleT sampling time (sec)

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

/*********************************************************************************
    get coefficients of 2nd order butterworth filter
**********************************************************************************/
void butterworth::get_coefficients(double cutoff_freq, double sample_freq) {
  /// Get coefficients of 2nd order butterworth filter
  /// b: numerator, a: denominator

  double T = 1.0 / sample_freq;

  double wc_a = 2.0 * M_PI * cutoff_freq;     // cutoff frequency (rad/s)
  double wc = 2.0 / T * tan(wc_a * T / 2.0);  // prewarp frequency

  double den = (T * T) * (wc * wc) + sqrt(2.0) * T * wc * 2.0 + 4.0;

  b(0) = (T * T) * (wc * wc) / den;
  b(1) = (T * T) * (wc * wc) * 2.0 / den;
  b(2) = (T * T) * (wc * wc) / den;

  a(0) = 1.0;
  a(1) = ((T * T) * (wc * wc) * 2.0 - 8.0) / den;
  a(2) = ((T * T) * (wc * wc) - sqrt(2.0) * T * wc * 2.0 + 4.0) / den;
}

/*********************************************************************************
    filter
**********************************************************************************/
double butterworth::filter(double x) {
  /// Butterworth filter implementation
  /// @param[in] x filter input (scalar)
  /// @param[out] y filter output (scalar)

  if (!firsttime) {  // avoid transient at the beginning
    // observe: b(0)+b(1)+b(2)-a(1)-a(2)=1
    // thus setting all internal variables to x makes initial output y=x
    // instead of setting all of them to zero

    vx(2) = x;
    vx(1) = x;
    vx(0) = x;

    vy(2) = x;
    vy(1) = x;
    vy(0) = x;

    firsttime = true;
  } else {
    vx(2) = vx(1);
    vx(1) = vx(0);
    vx(0) = x;

    vy(2) = vy(1);
    vy(1) = vy(0);
  }

  vy(0) = b(0) * vx(0) + b(1) * vx(1) + b(2) * vx(2) - a(1) * vy(1) - a(2) * vy(2);

  double y = vy(0);  // output

  if (fabs(y) < 1.0E-9)
    y = 0.0;

  return y;  // filter out
}
