#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H

#include <math.h>

#include <Eigen/Dense>
#include <vector>

namespace math_utility {

double sigmoid(double x, double gain);
double sigmoid(double x);

// check a point in a convex area
bool inConvex(Eigen::Vector2d p, std::vector<Eigen::Vector2d> apexes);

// clamp a value between min and max
template <class T>
T clamp(T value, T min, T max) {
  return std::min(std::max(min, value), max);
}

// return zero if a value is between upper and lower
template <class T>
T deadZone(T value, T upper, T lower) {
  if (value < upper && value > lower)
    value = 0.0;

  return value;
}

// return zero if a absolute value is less than upper
template <class T>
T deadZone(T value, T upper) {
  return deadZone(value, abs(upper), -abs(upper));
}

// retrun skew-symmetric (cross-product) matrix
template <typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec) {
  return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1), vec(2), T(0), -vec(0), -vec(1), vec(0), T(0)).finished();
}

// concate std::vector of Eigen::VectorXd
inline Eigen::VectorXd concatenateVecOfVec(std::vector<Eigen::VectorXd> vecOfVec) {
  int size = 0;
  for (size_t i = 0; i < vecOfVec.size(); i++)
    size += vecOfVec[i].size();

  Eigen::VectorXd vec(size);
  int start = 0;
  for (size_t i = 0; i < vecOfVec.size(); i++) {
    vec.segment(start, vecOfVec[i].size()) = vecOfVec[i];
    start += vecOfVec[i].size();
  }

  return vec;
}

// split Eigen::VectorXd into std::vector of Eigen::VectorXd
inline std::vector<Eigen::VectorXd> splitVec(Eigen::VectorXd vec, std::vector<int> num) {
  std::vector<Eigen::VectorXd> vecOfVec(num.size());
  int start = 0;
  for (size_t i = 0; i < num.size(); i++) {
    vecOfVec[i] = vec.segment(start, num[i]);
    start += num[i];
  }
  return vecOfVec;
}

// split Eigen::VectorXd into std::vector of Eigen::VectorXd
inline std::vector<Eigen::VectorXd> splitVec(Eigen::VectorXd vec, std::vector<Eigen::VectorXd>& vecOfVec) {
  std::vector<int> num(vecOfVec.size());
  for (size_t i = 0; i < vecOfVec.size(); i++)
    num[i] = vecOfVec[i].size();

  vecOfVec = splitVec(vec, num);
  return vecOfVec;
}

// retrun sgin of val (in input type)
template <typename T>
T sgn(T val) {
  return static_cast<T>((T(0) < val) - (val < T(0)));
}

// template <typename T>
inline Eigen::Quaterniond QuatFromTwoVectors(Eigen::Vector3d u, Eigen::Vector3d v) {
  double d = u.dot(v);
  Eigen::Vector3d w = u.cross(v);

  return Eigen::Quaterniond(d + sqrt(d * d + w.dot(w)), w(0), w(1), w(2)).normalized();
}

};  // namespace math_utility

namespace Eigen {
typedef Matrix<double, 1, 1> Vector1d;
typedef Matrix<double, 1, 1> Matrix1d;

typedef Matrix<float, 1, 1> Vector1f;
typedef Matrix<float, 1, 1> Matrix1f;
}  // namespace Eigen

#endif  // MATH_UTILITY_H
