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
