// Copyright 2018 Toyota Research Institute.  All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
    vec.push_back(lb + n * h);
  }
  return vec;
}

} // namespace linalg_utils
