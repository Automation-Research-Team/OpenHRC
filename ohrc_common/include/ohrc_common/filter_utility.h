#ifndef FILTER_UTILITY_H
#define FILTER_UTILITY_H


#include <vector>
#include <string>
#include "math.h"

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief The CFilter class
///
class CFilter {
public:
  CFilter(void);
  ~CFilter(void);
  int InitKFParam(void);

public:
  struct KALMAN_FILTER_PARAMETER {
    double fQ;
    double fR;
    double fX0;
    double fP0;
    double fXt;
    double fXt_1;
    double fPt;
    double fPt_1;
    double fKalman_Gain;
    double fKalman_Gain2;
    double fNewP;
  } m_Kalman_Param;

  struct ADMITTANCE_PARAM {
    double a1, b0, b1;
  } m_Admittance_Param;

  struct LPF_PARAM {
    double a1, a2, b0, b1, b2;
  } m_LPF_Param;

  double KF(double fInput);          // Kalman Flter
  double Admittance(double dbInput); // Low pass Filter (1���x���n)
  double LPF(double dbInput);        // Low Pass Filter
  void KFResetParam(double fInput);
  void InitAdmittanceParam(void);
  void AdmittanceParamSet(ADMITTANCE_PARAM *m_LPF1_Param, double Mv, double Dv, double sampling_freq);
  void LPFParamSet(LPF_PARAM *m_LPF_Param, double Zeta, double cutoff_freq, double sampling_freq);
  void LPFParamSetUV(void);
  std::vector<std::string> split(std::string str);
  double y1[2]={0.0}; // y[0]:y[k](0�X�e�b�v�O), y[1]:y[k-1](1�X�e�b�v�O)
  double u1[2]={0.0}; // u[0]:u[k](0�X�e�b�v�O), u[1]:y[k-1](1�X�e�b�v�O)
//  double y[3]={0.0};  // y[0]:y[k](0�X�e�b�v�O), y[1]:y[k-1](1�X�e�b�v�O), y[2]:y[k-2](2�X�e�b�v�O)
//  double u[3]={0.0};  // u[0]:u[k](0�X�e�b�v�O), u[1]:y[k-1](1�X�e�b�v�O), u[2]:u[k-2](2�X�e�b�v�O)
};
#if 0
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief The geometry_point_util class
///
class geometry_point_util {
private:
  double delta_t;
  CFilter _filter_x;
  CFilter _filter_y;
  CFilter _filter_z;
  void LPF_initialize(double Zeta = 0.7, double cutoff_freq = 50.0, double sampling_freq = 100.0);

public:
  void LPF(geometry_msgs::Point raw_point, geometry_msgs::Point &filtered_point);
  void diff(geometry_msgs::Point point, geometry_msgs::Point &diff_point);
  void diff_LPF(geometry_msgs::Point point, geometry_msgs::Point &filtered_point);
  double dist(geometry_msgs::Point point1, geometry_msgs::Point point2);
  geometry_point_util(double Zeta, double cutoff_freq, double sampling_freq);
};


class geometry_wrench_util {
private:
  double delta_t;
  CFilter _filter_x;
  CFilter _filter_y;
  CFilter _filter_z;
  CFilter _filter_tx;
  CFilter _filter_ty;
  CFilter _filter_tz;
  void LPF_initialize(double Zeta = 0.7, double cutoff_freq = 50.0, double sampling_freq = 100.0);

public:
  void LPF(geometry_msgs::Wrench raw_wrench, geometry_msgs::Wrench &filtered_wrench);
  void diff(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &diff_wrench);
  void diff_LPF(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &filtered_wrench);
  geometry_wrench_util(double Zeta, double cutoff_freq, double sampling_freq);
};
#endif

#endif // FILTER_UTILITY_H
