#pragma once

#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

template <class T> class State {
public:
  typedef Eigen::Matrix<T, 3, 1> Vec3t;

  State(const Vec3t &pos, const Vec3t &vel, const T yaw);
  ~State();

  // Accessors to the private members.
  Vec3t Pos() const;
  Vec3t Vel() const;
  T Yaw() const;

  // Print pos, vel and yaw information to terminal.
  void print() const;

private:
  Vec3t pos_;
  Vec3t vel_;
  T yaw_;
};

// Declare the explicit instantiations; i.e., the allowed implementations of the
// State class. You will only be able to use State with float or double.
template class State<float>;
template class State<double>;
