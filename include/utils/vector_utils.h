#pragma once

#include <algorithm>
#include <numeric>
#include <vector>

// Defines a collection of vector utility functions.
namespace vector_utils {

// Sorts a vector `v` of type `T` in ascending order, and outputs the sorted
// vector `v_sorted` and the sorted indices `idx_sorted`.
template <typename T>
void Sort(const std::vector<T> &v, std::vector<T> *v_sorted,
          std::vector<size_t> *idx_sorted) {

  // Initialize original index locations.
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // Sort indexes based on comparing values in v.
  std::sort(idx.begin(), idx.end(),
            [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

  // Populate the output parameters.
  *idx_sorted = idx;
  *v_sorted = v;
  std::sort(v_sorted->begin(), v_sorted->end());
}

} // namespace vector_utils
