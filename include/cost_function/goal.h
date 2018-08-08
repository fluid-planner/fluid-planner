#pragma once

#include <vector>

#include <cost_function/cost_function.h>
#include <trajectory/state.h>
#include <trajectory/trajectory.h>

namespace cost_function {

// `Goal` is an inherited class that computes cost based on a goal point.
class Goal : public CostFunction {
public:
  Goal();
  ~Goal();

  // Implementation of the virtual method from the parent class.
  std::vector<float>
  ComputeCost(std::vector<trajectory::Trajectory<float>> &trajs) override;

  // Sets the goal point to which the cost is evaluated.
  void SetGoal(const Eigen::Vector3f &goal);

private:
  Eigen::Vector3f goal_;  // The goal point.
  bool goal_set_ = false; // A flag to check whether goal has been set.
  // TODO(xuning@cmu.edu): Make this a parameter.
  const float dt = 0.2; // Time interval to evaluate the cost, in seconds.

  // An Euclidean distance function from `p1` to `p2`.
  float Dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);
};

} // namespace cost_function
