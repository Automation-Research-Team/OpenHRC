#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H

#include <math.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace math_utility {

// y = k / (1 + e^(-a*(x-x0)))+y0
inline double sigmoid(double x, double a, double x0, double y0, double k) {
  return k / (1.0 + std::exp(-a * (x - x0))) + y0;
}

inline double sigmoid(double x, double gain) {
  return sigmoid(x, gain, 0.0, 0.0, 1.0);
}

inline double sigmoid(double x) {
  return sigmoid(x, 1.0);
}

// check a point in a convex area
inline bool inConvex(Eigen::Vector2d p, std::vector<Eigen::Vector2d> apexes) {
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

class Integrator {
  inline Eigen::VectorXd EulerIntegrate(const double& t, const Eigen::VectorXd& x, const std::unordered_map<std::string, Eigen::MatrixXd>& u) {
    return x + dt * f(t, x, u);
  }

  inline Eigen::VectorXd RungeKuttaIntegrate(const double& t, const Eigen::VectorXd& x, const std::unordered_map<std::string, Eigen::MatrixXd>& u) {
    Eigen::VectorXd k1 = f(t, x, u);
    Eigen::VectorXd k2 = f(t + dt * 0.5, x + dt * 0.5 * k1, u);
    Eigen::VectorXd k3 = f(t + dt * 0.5, x + dt * 0.5 * k2, u);
    Eigen::VectorXd k4 = f(t + dt, x + dt * k3, u);

    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  const double dt;
  std::function<Eigen::VectorXd(const double& t, const Eigen::VectorXd&, const std::unordered_map<std::string, Eigen::MatrixXd>&)> f;

public:
  enum class Method { Euler, RungeKutta };
  Integrator(const Method method, std::function<Eigen::VectorXd(const double& t, const Eigen::VectorXd&, const std::unordered_map<std::string, Eigen::MatrixXd>&)> f, double dt)
    : dt(dt), f(f) {
    if (method == Method::Euler)
      integrate = [this](const double& t, const Eigen::VectorXd& x, const std::unordered_map<std::string, Eigen::MatrixXd>& u) { return this->EulerIntegrate(t, x, u); };
    else if (method == Method::RungeKutta)
      integrate = [this](const double& t, const Eigen::VectorXd& x, const std::unordered_map<std::string, Eigen::MatrixXd>& u) { return this->RungeKuttaIntegrate(t, x, u); };
    else
      throw std::invalid_argument("Invalid integration method");
  }
  std::function<Eigen::VectorXd(const double& t, const Eigen::VectorXd&, const std::unordered_map<std::string, Eigen::MatrixXd>&)> integrate;
};

};  // namespace math_utility

namespace Eigen {
typedef Matrix<double, 1, 1> Vector1d;
typedef Matrix<double, 1, 1> Matrix1d;

typedef Matrix<float, 1, 1> Vector1f;
typedef Matrix<float, 1, 1> Matrix1f;
}  // namespace Eigen

#endif  // MATH_UTILITY_H
