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

#include <trajectory/state.h>

namespace trajectory {

template <class T>
State<T>::State(const State<T>::Vec3t &pos, const State<T>::Vec3t &vel,
                const T yaw)
    : pos_(pos), vel_(vel), yaw_(yaw) {}

template <class T> State<T>::~State() {}

template <class T> typename State<T>::Vec3t State<T>::Pos() const {
  return pos_;
}

template <class T> typename State<T>::Vec3t State<T>::Vel() const {
  return vel_;
}

template <class T> T State<T>::Pos(size_t i) const {
  assert(i < 3);
  return pos_(i);
}

template <class T> T State<T>::Vel(size_t i) const {
  assert(i < 3);
  return vel_(i);
}

template <class T> T State<T>::Yaw() const { return yaw_; }

template <class T> void State<T>::print() const {
  printf("Position: [ %.2f, %.2f, %.2f ]
", pos_(0), pos_(1), pos_(2));
  printf("Velocity: [ %.2f, %.2f, %.2f ]
", vel_(0), vel_(1), vel_(2));
  printf("Yaw : %.2f
", yaw_);
}

} // namespace trajectory
