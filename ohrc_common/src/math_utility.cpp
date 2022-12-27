#include "ohrc_common/math_utility.h"

double math_utility::sigmoid(double x, double gain) {
  double y = 1.0 / (1.0 + pow(M_E, -gain * x));

  return y;
}

double math_utility::sigmoid(double x) {
  double gain = 1.0;
  double y = sigmoid(x, gain);

  return y;
}

bool math_utility::inConvex(Eigen::Vector2d p, std::vector<Eigen::Vector2d> apexes) {
  // extend 3d (add z componet for cross product below)
  Eigen::Vector3d p_3d = Eigen::Vector3d::Zero();
  p_3d.head(2) = p;

  std::vector<Eigen::Vector3d> apexes_3d(apexes.size(), Eigen::Vector3d::Zero());
  for (unsigned int i = 0; i < apexes_3d.size(); i++)
    apexes_3d[i].head(2) = apexes[i];

  int count = 0;
  apexes_3d.push_back(apexes_3d[0]);  // copy first vec at last
  for (unsigned int i = 0; i < apexes_3d.size() - 1; i++) {
    if ((p_3d - apexes_3d[i]).cross(apexes_3d[i + 1] - apexes_3d[i])(2) > 0)
      count++;
  }

  if (count == apexes_3d.size() - 1 || count == 0)  // all cross products are same sign (all positive or all negative)
    return true;
  else
    return false;
}

// bool math_utility::more_than(Eigen::VectorXd )
