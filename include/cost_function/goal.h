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
