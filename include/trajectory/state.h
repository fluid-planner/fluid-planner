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
  //  pos: a 3x1 vector that contains [x, y, z]. In 2D, z can also be interpreted
  //       to be the heading (depending on how you decide to use this field).
  //  vel: a 3x1 vector that contains [v_x, v_y, v_z]. In 2D, v_z is zero.
  //  yaw: a single T that contains the yaw. For 2D, this is the heading.
  State(const Vec3t &pos, const Vec3t &vel, const T yaw);
  ~State();

  // Accessors to the private members.
  Vec3t Pos() const;
  Vec3t Vel() const;
  T Yaw() const;

  // Print pos, vel and yaw information to terminal.
  void print() const;

private:
  Vec3t pos_;                   // Position.
  Vec3t vel_;                   // Velocity.
  T yaw_;                       // Yaw or heading, if in 2D.
};

// Declare the explicit instantiations; i.e., the allowed implementations of the
// State class. You will only be able to use State with float or double.
template class State<float>;
template class State<double>;

} // namespace trajectory
