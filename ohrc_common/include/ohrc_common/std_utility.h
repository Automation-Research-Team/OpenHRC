#ifndef STD_UTILITY_H
#define STD_UTILITY_H

#include <algorithm>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
namespace std_utility {

inline std::vector<std::vector<long long>> comb(long long n, long long r) {
  std::vector<bool> b(n);
  std::fill(b.end() - r, b.end(), true);

  std::vector<std::vector<long long>> combs = {};
  do {
    std::vector<long long> c = {};
    for (long long i = 0; i < n; i++)
      if (b[i])
        c.push_back(i);
    combs.push_back(c);
  } while (std::next_permutation(b.begin(), b.end()));
  return combs;
}

template <typename T>
int min_index(std::vector<T> vec) {
  auto minIt = std::min_element(std::begin(vec), std::end(vec));
  int minIndex = std::distance(std::begin(vec), minIt);

  return minIndex;
}

template <typename T>
int max_index(std::vector<T> vec) {
  auto minIt = std::max_element(std::begin(vec), std::end(vec));
  int minIndex = std::distance(std::begin(vec), minIt);
  return minIndex;
}

template <typename T, typename U>
inline U weightedSum(std::vector<T> weight, std::vector<U> vec) {
  U sum = weight[0] * vec[0];
  for (int i = 1; i < vec.size(); i++) {
    sum += weight[i] * vec[i];
  }
  return sum;
}

// compare abosolute values of input, and find max element.
// but return original value
template <typename _Tp>
_GLIBCXX14_CONSTEXPR inline _Tp max_abs(std::initializer_list<_Tp> __l) {
  return std::max(__l, [](_Tp v1, _Tp v2) { return std::abs(v1) < std::abs(v2); });
}

inline bool exist_in(const std::vector<std::string>& c, const std::string& v, int& index) {
  auto itr = std::find(c.begin(), c.end(), v);
  bool found = itr != c.end();

  if (found)
    index = std::distance(c.begin(), itr);
  else
    index = -1;

  return found;
}

inline bool exist_in(const std::vector<std::string>& c, const std::string& v) {
  int index;
  return exist_in(c, v, index);
}

inline std::string getDatetimeStr() {
  time_t t = time(nullptr);
  const tm* localTime = localtime(&t);
  std::stringstream s;
  s << localTime->tm_year + 1900;
  s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
  s << std::setw(2) << std::setfill('0') << localTime->tm_mday << "_";
  s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
  s << std::setw(2) << std::setfill('0') << localTime->tm_min;
  // s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

  return s.str();
}

inline Eigen::VectorXd concatenateVectors(std::vector<Eigen::VectorXd> vecs) {
  int s = 0, index = 0;
  for (auto vec : vecs) {
    s += vec.size();
  }
  Eigen::VectorXd vec(s);
  for (auto v : vecs) {
    vec.segment(index, v.size()) = v;
    index += v.size();
  }
  return vec;
}

inline Eigen::VectorXd concatenateVectors(std::vector<KDL::JntArray> vecs) {
  std::vector<Eigen::VectorXd> v(vecs.size());
  for (int i = 0; i < vecs.size(); i++) {
    v[i] = vecs[i].data;
  }
  return concatenateVectors(v);
}

template <typename T>
inline std::vector<T> makeVector(T in) {
  std::vector<T> out = { in };
  return out;
}

};  // namespace std_utility

#endif  // STD_UTILITY_H
