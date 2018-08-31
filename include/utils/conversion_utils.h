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

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Defines a collection of utility functions for converting vector types.
namespace conversion_utils {

// Converts a std::vector to an Eigen::Vector.
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> Vec2Eigen(const std::vector<T> &vec) {
  // Since we are passing in a const vector, we cannot use Eigen::Map to copy
  // memory directly. Thus, iterate over all elements.
  Eigen::Matrix<T, Eigen::Dynamic, 1> mat(vec.size());
  for (size_t i = 0; i < vec.size(); i++) {
    mat(i) = vec[i];
  }
  return mat;
}

// Converts an Eigen::Vector to a std::vector.
template <typename T>
std::vector<T> Eigen2Vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &mat) {
  std::vector<T> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
  return vec;
}

} // namespace conversion_utils
