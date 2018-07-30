#pragma once

#include <cassert>
#include <cmath>
#include <vector>

// Defines a collection of linear algebra utils.
namespace linalg_utils {

// Generates `N` linearly spaced values between `lb` and `ub`, inclusive.
template <typename T>
std::vector<T> Linspace(const T lb, const T ub, const int N) {
  // Check that the input arguments are acceptable.
  assert(N > 1);
  assert(ub > lb);

  // Generate the evenly spaced vector.
  T h = (ub - lb) / static_cast<T>(N - 1);
  std::vector<T> vec;
  for (int n = 0; n < N; n++) {
    vec.push_back(lb + n*h);
  }
  return vec;
}

} // namespace linalg_utils
