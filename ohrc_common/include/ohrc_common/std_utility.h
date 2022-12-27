#ifndef STD_UTILITY_H
#define STD_UTILITY_H

#include <algorithm>
#include <initializer_list>
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

bool exist_in(const std::vector<std::string>& c, const std::string& v, int& index);
bool exist_in(const std::vector<std::string>& c, const std::string& v);

std::string getDatetimeStr();
};  // namespace std_utility

#endif  // STD_UTILITY_H
