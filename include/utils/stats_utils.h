#pragma once

#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stats_utils {

// RangeSample draws k samples of type T from the range [lb, ub), and returns
// an Eigen::VectorXd object. T is one of { int, float, double }.
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> RangeSample(const T lb, const T ub,
                                                const int k) {

  // Generate uniform real distribution from lb to ub.
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> distribution(lb, ub);

  // Sample from range.
  auto samples = Eigen::Matrix<T, Eigen::Dynamic, 1>(k);
  for (int i = 0; i < k; i++) {
    samples[i] = distribution(generator);
  }
  return samples;
}

// DataSample draws k samples sampled uniformly at random, with replacement,
// from the data in `data`.
template <typename T>
std::vector<T> DataSample(const std::vector<T> &data, const int k) {
  size_t N = data.size();

  // Check that input arguments are valid.
  assert(N > 1);
  assert(k != 0);

  // Generate uniform integer distribution.
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_int_distribution<int> distribution(0, N - 1);

  // Sample from data.
  std::vector<T> sampled_data;
  for (int i = 0; i < k; i++) {
    int j = distribution(generator);
    sampled_data.push_back(data[j]);
  }
  return sampled_data;
}

// Discretesample draws k samples sampled at random according to distribution
// `prob` with replacement, where `prob` is a probability array whose elements
// sum to 1. T is one of { float, double }.
template <typename T>
std::vector<int> DiscreteSample(const std::vector<T> &prob, const int k) {
  size_t N = prob.size();

  // Check that the input arguments are valid.
  assert(N > 1);
  assert(k != 0);

  // Check that the sum of the probabilities is equal to 1.
  T prob_sum = 0;
  for (auto &p : prob)
    prob_sum += p;
  assert(std::fabs(prob_sum - 1.0) < std::numeric_limits<T>::min());

  // Create index vector.
  std::vector<int> idx;
  for (int i = 0; i <= N; i += 1) {
    idx.push_back(i);
  }
  assert(idx.size() == N + 1);

  // Generate piecewise constant distribution.
  std::random_device rd;
  std::mt19937 generator(rd());
  std::piecewise_constant_distribution<> distribution(idx.begin(), idx.end(),
                                                      prob.begin());

  // Sample according to the probabilities.
  std::vector<int> sampled_idx;
  for (int i = 0; i < k; i += 1) {
    sampled_idx.push_back(std::floor(distribution(generator)));
  }
  return sampled_idx;
}
}
