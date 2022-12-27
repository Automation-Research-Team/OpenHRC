/**
 * @file filter_utility.cpp
 * @author Shunki Itadera
 * @date Oct. 2018
 * @brief utility library for filiter
 **/

#include <ohrc_common/filter_utility.h>

CFilter::CFilter(void) {
}

CFilter::~CFilter(void) {
}

int CFilter::InitKFParam(void) {
  m_Kalman_Param.fQ = 0.0001;
  m_Kalman_Param.fR = 0.10;
  // m_Kalman_Param.fX0			=	0.358887;
  m_Kalman_Param.fXt_1 = 0.0;
  m_Kalman_Param.fP0 = 0.0;
  m_Kalman_Param.fPt_1 = 0.0;
  m_Kalman_Param.fKalman_Gain = 0.0;
  return 0;
}

double CFilter::KF(double fInput) {
  // m_Kalman_Param.fPt			=	m_Kalman_Param.fPt_1 + m_Kalman_Param.fQ;
  m_Kalman_Param.fKalman_Gain = 0.10;
  // m_Kalman_Param.fKalman_Gain	=	(m_Kalman_Param.fPt	+ m_Kalman_Param.fQ)/ (m_Kalman_Param.fPt +	m_Kalman_Param.fQ + m_Kalman_Param.fR);
  // m_Kalman_Param.fPt			=	(m_Kalman_Param.fPt	+ m_Kalman_Param.fQ)/ (m_Kalman_Param.fPt +	m_Kalman_Param.fQ +
  // m_Kalman_Param.fR)*m_Kalman_Param.fR;
  m_Kalman_Param.fXt = m_Kalman_Param.fXt_1 + m_Kalman_Param.fKalman_Gain * (fInput - m_Kalman_Param.fXt_1);
  // m_Kalman_Param.fNewP		=	(1 - m_Kalman_Param.fKalman_Gain) * m_Kalman_Param.fPt;
  // m_Kalman_Param.fPt_1		=	m_Kalman_Param.fNewP;
  m_Kalman_Param.fXt_1 = m_Kalman_Param.fXt;
  return m_Kalman_Param.fXt;
}

void CFilter::KFResetParam(double fInput) {
  m_Kalman_Param.fXt_1 = fInput;
}

void CFilter::InitAdmittanceParam() {
  for (int i = 0; i < 2; ++i) {
    u1[i] = 0;
    y1[i] = 0;
  }
}

void CFilter::AdmittanceParamSet(ADMITTANCE_PARAM *m_Admittance_Param, double Mv, double Dv, double sampling_freq) {
  double T = 1.0 / sampling_freq;  //
  double gamma = Dv * T + 2.0 * Mv;
  m_Admittance_Param->a1 = (Dv * T - 2.0 * Mv) / gamma;
  m_Admittance_Param->b0 = T / gamma;
  m_Admittance_Param->b1 = T / gamma;
}

void CFilter::LPFParamSet(LPF_PARAM *m_LPF_Param, double Zeta, double cutoff_freq, double sampling_freq) {
  double T = 1.0 / sampling_freq;  //
  double gamma = 0.5 * T * 2.0 * M_PI * cutoff_freq;
  ////////////////////////////////////////////////////////////////////////////////////////////////

  m_LPF_Param->a1 = 2.0 * (gamma * gamma - 1.0) / (1.0 + 2.0 * Zeta * gamma + gamma * gamma);
  m_LPF_Param->a2 = (1.0 - 2.0 * Zeta * gamma + gamma * gamma) / (1.0 + 2.0 * Zeta * gamma + gamma * gamma);
  m_LPF_Param->b0 = (gamma * gamma) / (1.0 + 2.0 * Zeta * gamma + gamma * gamma);
  m_LPF_Param->b1 = (2.0 * gamma * gamma) / (1.0 + 2.0 * Zeta * gamma + gamma * gamma);
  m_LPF_Param->b2 = (gamma * gamma) / (1.0 + 2.0 * Zeta * gamma + gamma * gamma);
}

// not used
void CFilter::LPFParamSetUV(void) {
  //  for (int i = 0; i < 3; ++i) {
  //    u[i] = 0;
  //    y[i] = 0;
  //  }
}

double CFilter::Admittance(double dbInput) {
  double dbRet;
  //////////////////////////////
  u1[1] = u1[0];
  if (fabs(dbInput) < 0.000000001) {
    u1[0] = 0;
  } else {
    u1[0] = dbInput;
  }

  //////////////////////////////////////////
  dbRet = -m_Admittance_Param.a1 * y1[1] + m_Admittance_Param.b0 * u1[0] + m_Admittance_Param.b1 * u1[1];
  if (fabs(dbRet) < 0.000000001) {
    dbRet = 0;
  }

  //!////////////////////////////
  y1[1] = dbRet;
  y1[0] = 0;

  //////////////////////////////////////////

  return dbRet;
}

// IIR (Infinit Impulse Response) filter
double CFilter::LPF(double dbInput) {
  static double y[3] = { 0.0 };  // y[0]:y[k](0�X�e�b�v�O), y[1]:y[k-1](1�X�e�b�v�O), y[2]:y[k-2](2�X�e�b�v�O)
  static double u[3] = { 0.0 };  // u[0]:u[k](0�X�e�b�v�O), u[1]:y[k-1](1�X�e�b�v�O), u[2]:u[k-2](2�X�e�b�v�O)

  u[2] = u[1];
  u[1] = u[0];
  //  if (fabs(dbInput) < 0.000000001) {
  //    u[0] = 0;
  //  }else{
  u[0] = dbInput;
  //  }
  //////////////////////////////////////////
  double dbRet = -m_LPF_Param.a1 * y[1] - m_LPF_Param.a2 * y[2] + m_LPF_Param.b0 * u[0] + m_LPF_Param.b1 * u[1] + m_LPF_Param.b2 * u[2];
  //  if (fabs(dbRet) < 0.000000001)  {
  //    dbRet = 0;
  //  }

  //////////////////////////////
  y[2] = y[1];
  y[1] = dbRet;

  y[0] = 0.0;
  //////////////////////////////////////////

  return dbRet;
}

// 文字列の分断

std::vector<std::string> CFilter::split(std::string str) {
  std::string delim = "|";
  std::vector<std::string> items;
  std::size_t dlm_idx;

  if (str.npos == (dlm_idx = str.find_first_of(delim))) {
    items.push_back(str.substr(0, dlm_idx));
  }

  while (str.npos != (dlm_idx = str.find_first_of(delim))) {
    if (str.npos == str.find_first_not_of(delim)) {
      break;
    }

    items.push_back(str.substr(0, dlm_idx));
    dlm_idx++;
    str = str.erase(0, dlm_idx);

    if (str.npos == str.find_first_of(delim) && "" != str) {
      items.push_back(str);
      break;
    }
  }

  return items;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if 0
void geometry_point_util::LPF_initialize(double Zeta, double cutoff_freq, double sampling_freq) {
  _filter_x.LPFParamSetUV();
  _filter_y.LPFParamSetUV();
  _filter_z.LPFParamSetUV();
  _filter_x.LPFParamSet(&_filter_x.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_y.LPFParamSet(&_filter_y.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_z.LPFParamSet(&_filter_z.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
}

void geometry_point_util::LPF(geometry_msgs::Point raw_point, geometry_msgs::Point &filtered_point) {
  filtered_point.x = _filter_x.LPF(raw_point.x);
  filtered_point.y = _filter_y.LPF(raw_point.y);
  filtered_point.z = _filter_z.LPF(raw_point.z);
}

void geometry_point_util::diff(geometry_msgs::Point point, geometry_msgs::Point &diff_point) {
  static geometry_msgs::Point old_point = point;
  diff_point.x                          = (point.x - old_point.x) / delta_t;
  diff_point.y                          = (point.y - old_point.y) / delta_t;
  diff_point.z                          = (point.z - old_point.z) / delta_t;
  old_point                             = point;
}

void geometry_point_util::diff_LPF(geometry_msgs::Point point, geometry_msgs::Point &filtered_point) {
  geometry_msgs::Point diff_point;
  diff(point, diff_point);
  LPF(diff_point, filtered_point);
}

double geometry_point_util::dist(geometry_msgs::Point point1, geometry_msgs::Point point2) {
  return sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0) + pow(point1.z - point2.z, 2.0));
}

geometry_point_util::geometry_point_util(double Zeta, double cutoff_freq, double sampling_freq) {
  LPF_initialize(Zeta, cutoff_freq, sampling_freq);
  delta_t = 1.0 / sampling_freq;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void geometry_wrench_util::LPF_initialize(double Zeta, double cutoff_freq, double sampling_freq) {
  _filter_x.LPFParamSetUV();
  _filter_y.LPFParamSetUV();
  _filter_z.LPFParamSetUV();
  _filter_x.LPFParamSet(&_filter_x.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_y.LPFParamSet(&_filter_y.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_z.LPFParamSet(&_filter_z.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);

  _filter_tx.LPFParamSetUV();
  _filter_ty.LPFParamSetUV();
  _filter_tz.LPFParamSetUV();
  _filter_tx.LPFParamSet(&_filter_x.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_ty.LPFParamSet(&_filter_y.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
  _filter_tz.LPFParamSet(&_filter_z.m_LPF_Param, Zeta, cutoff_freq, sampling_freq);
}

void geometry_wrench_util::LPF(geometry_msgs::Wrench raw_wrench, geometry_msgs::Wrench &filtered_wrench) {
  filtered_wrench.force.x  = _filter_x.LPF(raw_wrench.force.x);
  filtered_wrench.force.y  = _filter_y.LPF(raw_wrench.force.y);
  filtered_wrench.force.z  = _filter_z.LPF(raw_wrench.force.z);
  filtered_wrench.torque.x = _filter_tx.LPF(raw_wrench.torque.x);
  filtered_wrench.torque.y = _filter_ty.LPF(raw_wrench.torque.y);
  filtered_wrench.torque.z = _filter_tz.LPF(raw_wrench.torque.z);
}

void geometry_wrench_util::diff(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &diff_wrench) {
  static geometry_msgs::Wrench old_wrench = wrench;

  diff_wrench.force.x  = (wrench.force.x - old_wrench.force.x) / delta_t;
  diff_wrench.force.y  = (wrench.force.y - old_wrench.force.y) / delta_t;
  diff_wrench.force.z  = (wrench.force.z - old_wrench.force.z) / delta_t;
  diff_wrench.torque.x = (wrench.torque.x - old_wrench.torque.x) / delta_t;
  diff_wrench.torque.y = (wrench.torque.y - old_wrench.torque.y) / delta_t;
  diff_wrench.torque.z = (wrench.torque.z - old_wrench.torque.z) / delta_t;

  old_wrench = wrench;
}

void geometry_wrench_util::diff_LPF(geometry_msgs::Wrench wrench, geometry_msgs::Wrench &filtered_wrench) {
  geometry_msgs::Wrench diff_wrench;
  diff(wrench, diff_wrench);
  LPF(diff_wrench, filtered_wrench);
}

geometry_wrench_util::geometry_wrench_util(double Zeta, double cutoff_freq, double sampling_freq) {
  LPF_initialize(Zeta, cutoff_freq, sampling_freq);
  delta_t = 1.0 / sampling_freq;
}
#endif
