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
