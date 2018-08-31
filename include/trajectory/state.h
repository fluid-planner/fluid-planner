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

#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace trajectory {

// Defines a simple class for the state containing pos, vel, and yaw,
// without frame specific definitions.
template <class T> class State {
public:
  typedef Eigen::Matrix<T, 3, 1> Vec3t;

  // Constructor parameters:
  //  pos: a 3x1 vector that contains [x, y, z]. In 2D, z can also be
  //  interpreted
  //       to be the heading (depending on how you decide to use this field).
  //  vel: a 3x1 vector that contains [v_x, v_y, v_z]. In 2D, v_z is zero.
  //  yaw: a single T that contains the yaw. For 2D, this is the heading.
  State(const Vec3t &pos, const Vec3t &vel, const T yaw);
  ~State();

  // Accessors to the private members.
  Vec3t Pos() const;
  Vec3t Vel() const;
  T Pos(size_t i) const;
  T Vel(size_t i) const;
  T Yaw() const;

  // Print pos, vel and yaw information to terminal.
  void print() const;

private:
  Vec3t pos_; // Position.
  Vec3t vel_; // Velocity.
  T yaw_;     // Yaw or heading, if in 2D.
};

// Declare the explicit instantiations; i.e., the allowed implementations of the
// State class. You will only be able to use State with float or double.
template class State<float>;
template class State<double>;

} // namespace trajectory
