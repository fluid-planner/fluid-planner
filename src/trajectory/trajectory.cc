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

#include <trajectory/trajectory.h>

namespace cu = conversion_utils;

namespace trajectory {

template <class T> Trajectory<T>::~Trajectory() {}

template <class T>
Trajectory<T>::Trajectory(const VecXt &vels, const VecXt &omegas,
                          const float seg_duration)
    : vels_(vels), omegas_(omegas), seg_duration_(seg_duration) {
  // Check that the parameters vels and omegas have the correct dimensions.
  assert(vels.size() != 0);
  assert(omegas.size() != 0);
  assert(vels.size() == omegas.size());

  num_seg_ = vels.size();
}

template <class T>
Trajectory<T>::Trajectory(const std::vector<T> &vels,
                          const std::vector<T> &omegas,
                          const float seg_duration) {
  // Check that the parameters vels and omegas has the correct dimensions.
  assert(vels.size() != 0);
  assert(omegas.size() != 0);
  assert(vels.size() == omegas.size());

  num_seg_ = vels.size();
  seg_duration_ = seg_duration;

  vels_ = cu::Vec2Eigen<T>(vels);
  omegas_ = cu::Vec2Eigen<T>(omegas);
}

template <class T>
Trajectory<T>::Trajectory(const Trajectory<T>::VecXt &Z,
                          const float seg_duration) {
  // Check that the parameter input Z = [vels, omegas] is of correct size.
  assert(Z.size() != 0);
  assert(Z.size() % 2 == 0);

  num_seg_ = Z.size() * 0.5;
  seg_duration_ = seg_duration;

  vels_ = Z.block(0, 0, num_seg_, 1);
  omegas_ = Z.block(num_seg_, 0, num_seg_, 1);
}

template <class T> typename Trajectory<T>::VecXt Trajectory<T>::Z() const {
  Trajectory<T>::VecXt Z(num_seg_ * 2);
  Z.block(0, 0, num_seg_, 1) = vels_;
  Z.block(num_seg_, 0, num_seg_, 1) = omegas_;
  return Z;
}

template <class T>
void Trajectory<T>::SetTransform(const Eigen::Transform<T, 3, 3> &R_wb,
                                 const Eigen::Matrix<T, 3, 1> &T_wb) {
  R_wb_ = R_wb;
  T_wb_ = T_wb;
  is_transform_init_ = true;
}

template <class T>
std::vector<State<T>> Trajectory<T>::GetWaypointsWorld(const float dt) {
  MatXt pos, vel;
  GetWaypointsWorld(dt, &pos, &vel);

  std::vector<State<T>> wpts;
  for (int i = 0; i < pos.cols(); i++) {
    State<T> state(pos.col(i), vel.col(i), pos(2, i));
    wpts.push_back(state);
  }
  return wpts;
}

template <class T>
std::vector<State<T>> Trajectory<T>::GetWaypointsLocal(const float dt) {
  MatXt pos, vel;
  GetWaypointsLocal(dt, &pos, &vel);

  std::vector<State<T>> wpts;
  for (int i = 0; i < pos.cols(); i++) {
    State<T> state(pos.col(i), vel.col(i), pos(2, i));
    wpts.push_back(state);
  }
  return wpts;
}

template <class T>
void Trajectory<T>::GetWaypointsWorld(const float dt, MatXt *pos, MatXt *vel) {

  if (!is_transform_init_) {
    printf("[Trajectory::GetWaypointsWorld] Warning: transforms not "
           "initialized. Call SetTransform before calling "
           "GetWaypointsWorld.
");
    return;
  }

  MatXt pos_b;
  MatXt vel_b;
  GetWaypointsLocal(dt, &pos_b, &vel_b);

  size_t n = pos_b.cols();
  pos->resize(3, n);
  vel->resize(3, n);

  for (size_t i = 0; i < n; i++) {
    pos->col(i) = R_wb_ * pos_b.col(i) + T_wb_;
    vel->col(i) = R_wb_ * vel_b.col(i);
  }
}

template <class T>
void Trajectory<T>::GetWaypointsLocal(const float dt, MatXt *pos, MatXt *vel) {
  size_t num_pts_per_seg = std::ceil(seg_duration_ / dt);
  int num_pts = num_seg_ * num_pts_per_seg;
  pos->resize(3, num_pts);
  vel->resize(3, num_pts);
  Trajectory<T>::Vec3t pos_prev = Trajectory<T>::Vec3t::Zero();
  int j = 0;
  float t = 0;

  for (int i = 0; i < num_seg_; i++) {
    T v = vels_(i);
    T omega = omegas_(i);

    for (int k = 0; k < num_pts_per_seg; k++) {
      t = k * dt;
      pos->col(j) = Trajectory<T>::Dubins(v, omega, t, pos_prev);
      vel->col(j) =
          Eigen::AngleAxis<T>((*pos)(2, j), Trajectory<T>::Vec3t(0, 0, 1)) *
          Trajectory<T>::Vec3t(v, 0, 0);
      j++;
    }
    pos_prev = pos->col(j - 1);
  }
}

template <class T>
typename Trajectory<T>::Vec3t
Trajectory<T>::Dubins(const T v, const T omega, const float t,
                      const Trajectory<T>::Vec3t &pos_prev) {
  Trajectory<T>::Vec3t x;
  T theta = pos_prev(2);
  if (std::fabs(omega) < std::numeric_limits<T>::min()) {
    x = Trajectory<T>::Vec3t(v * t * cos(theta), v * t * sin(theta), 0.0) +
        pos_prev;
  } else {
    x = Trajectory<T>::Vec3t(v / omega * (sin(theta + t * omega) - sin(theta)),
                             v / omega * (-cos(theta + t * omega) + cos(theta)),
                             t * omega) +
        pos_prev;
  }
  return x;
}

} // namespace trajectory
