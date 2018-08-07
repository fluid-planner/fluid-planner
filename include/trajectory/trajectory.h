#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <trajectory/state.h>
#include <utils/conversion_utils.h>

// Defines a collection of classes associated with trajectory definitions.
namespace trajectory {

// Defines a Trajectory class, which represents an m-segment trajectory
// parameterized by linear velocity `v` and angular velocity `omega`.
template <class T> class Trajectory {
public:
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VecXt;
  typedef Eigen::Matrix<T, 3, Eigen::Dynamic> MatXt;
  typedef Eigen::Matrix<T, 3, 1> Vec3t;

  // Constructor parameters:
  //  vels: an m x 1 vector containing linear velocities for each segment
  //  omegas: an m x 1 vector containing angular velocities for each segment
  // OR:
  //  Z: an 2m x 1 vector containing [vels, omegas]
  // AND:
  //  seg_duration: a float denoting the duration of each segment (in seconds).
  Trajectory(const std::vector<T> &vels, const std::vector<T> &omegas,
             const float seg_duration);
  Trajectory(const VecXt &vels, const VecXt &omegas, const float seg_duration);
  Trajectory(const VecXt &Z, const float seg_duration);
  ~Trajectory();

  // Sets the Rotation and Translation fields that transforms the trajectory
  // from local to world frame.
  void SetTransform(const Eigen::Transform<T, 3, 3> &R_wb, const Vec3t &T_wb);

  // Returns the Z representation of this trajectory; i.e., a 2m x 1 vector that
  // contains the parameters to the trajectory [vels, omegas].
  VecXt Z() const;

  // Returns a discretized path representation of this trajectory as a vector of
  // States, discretized according to `dt`.
  std::vector<State<T>> GetWaypointsWorld(const float dt);
  std::vector<State<T>> GetWaypointsLocal(const float dt);

  // Outputs a discretized path representation of this trajectory as two Eigen
  // Matrices, `pos` and `vel`.
  void GetWaypointsWorld(const float dt, MatXt *pos, MatXt *vel);
  void GetWaypointsLocal(const float dt, MatXt *pos, MatXt *vel);

private:
  int num_seg_;   // Number of segments, m.
  int num_param_; // Number of parameters, which should be 2m.

  VecXt vels_;         // Linear velocity parameters.
  VecXt omegas_;       // Angular velocity parameters.
  float seg_duration_; // Duration of each segment.

  bool is_transform_init_ = false; // A flag indicating if R_wb_ and T_wb_ are
                                   // initialized.
  Eigen::Transform<T, 3, 3> R_wb_; // Rotation from local to world frame.
  Vec3t T_wb_;                     // Translation from local to world frame.

  // Dubins model parameters:
  //  v:        linear velocity.
  //  omega:    angular velocity.
  //  t:        evaluation time.
  //  pos_prev: previous position.
  // Output:
  //  A 3x1 vector of the position x.
  Vec3t Dubins(const T v, const T omega, const float t, const Vec3t &pos_prev);
};

// Declare the explicit instantiations; i.e., the allowed implementations of the
// Trajectory class. You will only be able to use Trajectory with float or
// double, but time in all of these instantiations will be represented by a
// float.
template class Trajectory<float>;
template class Trajectory<double>;

} // namespace trajectory
