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
