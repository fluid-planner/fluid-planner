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

#include <cost_function/goal.h>

#include <iostream>

namespace tr = trajectory;

namespace cost_function {

Goal::Goal() {}
Goal::~Goal() {}

std::vector<float>
Goal::ComputeCost(std::vector<tr::Trajectory<float>> &trajs) {
  std::vector<float> costs;
  if (!goal_set_) {
    std::cerr << "Goal not set; cannot compute cost! Exiting." << std::endl;
    return costs;
  }

  for (auto &tj : trajs) {
    std::vector<tr::State<float>> wpts = tj.GetWaypointsWorld(dt);
    float cost = 0;
    for (size_t i = 0; i < wpts.size(); i++) {
      cost += Dist(wpts[i].Pos(), goal_);
    }
    costs.push_back(cost);
  }
  return costs;
}

void Goal::SetGoal(const Eigen::Vector3f &goal) {
  goal_ = goal;
  goal_set_ = true;
}

float Goal::Dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) {
  return (p1 - p2).norm();
}

} // namespace cost_function
