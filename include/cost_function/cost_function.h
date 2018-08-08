#pragma once

#include <vector>

#include <trajectory/trajectory.h>

namespace cost_function {

// `CostFunction` is an abstract parent class that contains only the virtual
// function `ComputeCost`, which is called by the planner.
class CostFunction {
public:
  CostFunction(){};
  ~CostFunction(){};

  // Computes cost of a set of trajectories `trajs`, and returns them in a
  // vector.
  virtual std::vector<float>
  ComputeCost(std::vector<trajectory::Trajectory<float>> &trajs) = 0;
};

} // namespace cost_function
