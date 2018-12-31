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
#include <limits>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Defines a collection of statistics utility functions.
namespace stats_utils {

// Define a sufficiently small value, but
static constexpr float kSufficientlySmallFloat = 0.00001;

template <typename T> using VecXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
template <typename T>
using MatXt = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// RangeSample draws k samples of type T from the range [lb, ub), and returns
// an Eigen::VectorXd object. T is one of { int, float, double }.
template <typename T>
VecXt<T> RangeSample(const T lb, const T ub, const int k) {

  // Generate uniform real distribution from lb to ub.
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> distribution(lb, ub);

  // Sample from range.
  auto samples = VecXt<T>(k);
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

// DiscreteSample draws k samples sampled at random according to distribution
// `prob` with replacement, where `prob` is a probability array whose elements
// sum to 1. T is one of { float, double }.
template <typename T>
std::vector<int> DiscreteSample(const std::vector<T> &prob, const int k) {
  size_t N = prob.size();

  // Check that the input arguments are valid.
  assert(N > 1);
  assert(k != 0);

  // Check that the sum of the probabilities is equal to 1 (accommodating
  // rounding errors).
  T prob_sum = 0;
  for (auto &p : prob)
    prob_sum += p;
  assert(std::abs(prob_sum - 1.0) < kSufficientlySmallFloat);

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

// GaussianPdf computes the probability density for a sample `x`, according to
// mean `mean` and covariance `sigma`.
// Inputs:
//  x:      vector of size M
//  mean:   vector of size M
//  sigma:  matrix of size MxM
template <typename T>
T GaussianPdf(const VecXt<T> &x, const VecXt<T> &mean, const MatXt<T> &sigma) {
  assert(x.rows() == mean.rows());
  assert(x.cols() == mean.cols());
  assert(sigma.rows() == sigma.cols());

  // Precompute constants.
  int n = std::max(x.rows(), x.cols());
  const T log_sqrt_2pi = 0.5 * std::log(2 * M_PI);

  // Use Cholesky decomposition so as to prevent numerical instability in
  // computing the inverse and determinant.
  Eigen::LLT<MatXt<T>> chol(sigma);

  if (chol.info() == Eigen::NumericalIssue)
    throw std::runtime_error("Possibly non PSD matrix!");

  const typename Eigen::LLT<MatXt<T>>::Traits::MatrixL &L = chol.matrixL();
  T dist = (L.solve(x - mean)).squaredNorm();
  return std::exp(-n * log_sqrt_2pi - 0.5 * dist) / L.determinant();
}

// Covariance computes the sample covariance for an Eigen Matrix `samples` of
// size MxN, and outputs an Eigen matrix of size MxM.
// `samples` contains N samples of size M.
template <typename T> MatXt<T> Covariance(const MatXt<T> &samples) {
  VecXt<T> mean = samples.rowwise().mean();
  return Covariance(samples, mean);
}

// Covariance computes the sample covariance for an Eigen Matrix `samples` of
// size MxN according to the mean `mean`, and outputs an Eigen matrix of size
// MxM. `samples` contains N samples of size M.
template <typename T>
MatXt<T> Covariance(const MatXt<T> &samples, const VecXt<T> &mean) {
  int N = samples.cols();
  int sample_dim = mean.size();
  assert(samples.rows() == sample_dim);

  MatXt<T> centered = samples.colwise() - mean;
  MatXt<T> cov = (centered * centered.transpose()) / N;

  // Check that the sizes are correct.
  assert(cov.rows() == sample_dim);
  assert(cov.cols() == sample_dim);
  return cov;
}

// Covariance computes the weighted covariance for an Eigen Matrix `samples` of
// size MxN according to the mean `mean`, and outputs an Eigen matrix of size
// MxM. `samples` contains N samples of size M.
template <typename T>
MatXt<T> Covariance(const MatXt<T> &samples, const VecXt<T> &mean,
                    const VecXt<T> &weights) {

  int N = samples.cols();
  int sample_dim = mean.size();
  assert(samples.rows() == sample_dim);
  assert(weights.size() == N);

  MatXt<T> centered = samples.colwise() - mean;
  MatXt<T> weighted_centered =
      centered.array().rowwise() * weights.transpose().array();
  MatXt<T> cov = (centered * weighted_centered.transpose()) / weights.sum();

  // Check that the sizes are correct.
  assert(cov.rows() == sample_dim);
  assert(cov.cols() == sample_dim);
  return cov;
}

} // namespace stats_utils
