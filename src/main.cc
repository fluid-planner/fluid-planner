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

#include <chrono>
#include <iostream>
#include <vector>

#include <fluid_planner.h>

#include <cost_function/cost_function.h>
#include <cost_function/flow_field.h>
#include <cost_function/goal.h>
#include <trajectory/state.h>
#include <trajectory/trajectory.h>
#include <utils/conversion_utils.h>
#include <utils/data_handling.h>
#include <utils/stats_utils.h>

namespace tr = trajectory;
namespace cf = cost_function;
namespace fp = fluid_planner;
namespace cu = conversion_utils;
namespace su = stats_utils;

// Given a matrix of data, `GetTrajectoryInLocalFrame` turns the segment of
// waypoints into a vector of trajectory::State.
std::vector<tr::State<float>>
GetTrajectoryInLocalFrame(const Eigen::MatrixXf &data) {
  std::vector<tr::State<float>> traj;
  // Need to also transform this into the local frame.
  Eigen::Matrix3f R_wb;
  R_wb = Eigen::AngleAxisf(data(3, 0), Eigen::Vector3f::UnitZ());
  Eigen::Vector3f T_wb = Eigen::Vector3f(data(0, 0), data(1, 0), data(3, 0));
  for (size_t i = 0; i < data.cols(); i++) {
    float yaw = data(3, i);
    Eigen::Vector3f pos =
        R_wb.transpose() *
        (Eigen::Vector3f(data(0, i), data(1, i), data(3, i)) - T_wb);
    Eigen::Vector3f vel = R_wb.transpose() *
                          Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                          Eigen::Vector3f(data(5, i), 0, 0);
    traj.push_back(tr::State<float>(pos, vel, yaw));
  }
  return traj;
}

// An example usage of the FLUID planner.
int main() {
  // Load data from csv.
  Eigen::MatrixXf data = data_handling::LoadCsv("../data/data1.csv", 8, 14014);
  float dt = 0.008;
  std::deque<tr::State<float>> training_examples;

  // Parameters
  fp::TrajProperties tp;
  tp.num_seg = 2;
  tp.traj_duration = 3;
  tp.v_ub = 60;
  tp.v_lb = 0;
  tp.omega_lb = -1;
  tp.omega_ub = 1;
  tp.num_action_space_discretizations = 60;

  fp::PlannerProperties pp;
  pp.sample_dev = 0.1;
  pp.num_iter = 20;
  pp.rho = 0.1;
  pp.k = 1;
  pp.num_traj = 30;

  std::string cp_type = "FLOW";

  cf::GP gp;
  gp.dim = 2;
  gp.cov_kernel = "CovSum( CovSEiso, CovNoise)";
  gp.hyp_params_x = std::vector<double>{1.2, 4.1, 4.0};
  gp.hyp_params_y = std::vector<double>{1.0, 1.5, 4.0};
  size_t window_size = 6;

  // Initialize replan window.
  float replan_every_s = 0.5;
  int replan_dt = std::round(replan_every_s / dt);

  // Flow Field segment duration length.
  float max_seg_duration = 5;
  int dS = std::round(max_seg_duration / dt);

  // Segment goal, sampled from some time in the future.
  float goal_length = tp.traj_duration;
  int dN = std::round(goal_length / dt);

  /* ====================================================================== */

  fp::FluidPlanner flp;
  flp.Initialize(pp, tp);

  cf::FlowField ff_cf;
  cf::Goal goal_cf;

  if (cp_type == "FLOW") {
    ff_cf.Initialize(gp, window_size);
  }

  // Open files to write to.
  std::ofstream file_traj, file_traj_world, file_traj_local, file_goal,
      file_field, file_meta;
  file_traj_world.open("traj_world.csv");
  file_traj_local.open("traj_local.csv");
  file_traj.open("traj.csv");
  file_field.open("field.csv");
  file_meta.open("meta.csv");
  if (cp_type == "GOAL")
    file_goal.open("goal.csv");

  /* ====================================================================== */

  std::cout << "Starting planner..." << std::endl;

  for (int i = 0; i < dN * 15 - dN; i += replan_dt) {
    std::cout << "Planner time stamp: " << i * dt << "s" << std::endl;

    // Update the GOAL cost function if needed. goal = [X, Y, heading]
    if (cp_type == "GOAL") {
      Eigen::Vector3f goal_i =
          Eigen::Vector3f(data(0, i + dN), data(1, i + dN), data(3, i + dN));
      goal_cf.SetGoal(goal_i);
      data_handling::WritePointToFile(goal_i, file_goal);
    }

    float yaw = data(3, i);
    Eigen::Vector3f start_pos(
        Eigen::Vector3f(data(0, i), data(1, i), data(3, i)));
    Eigen::Vector3f start_vel(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                              Eigen::Vector3f(data(5, i), 0, 0));
    tr::State<float> start(start_pos, start_vel, yaw);

    // Plan!
    std::vector<tr::Trajectory<float>> traj = flp.Plan(start, ff_cf);

    // Update FLOW cost function.
    if (cp_type == "FLOW") {
      printf("Updating Flow Field cost function.");
      auto t1 = std::chrono::high_resolution_clock::now();

      ff_cf.Clear();
      int seg_len = std::min(i, dS);
      if (seg_len == 0)
        continue;
      Eigen::MatrixXf data_segment = data.block(0, i - seg_len, 6, seg_len);
      std::vector<tr::State<float>> odom_segment =
          GetTrajectoryInLocalFrame(data_segment);
      ff_cf.AddSegment(odom_segment);

      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<float> dur1 = t2 - t1;
      printf(" adding seg: %.4fs", dur1.count());

      ff_cf.Learn();

      auto t3 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<float> dur2 = t3 - t2;
      printf(" learning cf: %.4fs
", dur2.count());
    }

    // Write to file.
    std::cout << "Writing meta data to file." << std::endl;
    file_meta << i * dt << "," << traj.size() << std::endl;

    std::cout << "Writing trajectories to file." << std::endl;

    data_handling::WriteWptsToFile(traj, file_traj_world, "WORLD");
    data_handling::WriteWptsToFile(traj, file_traj_local, "LOCAL");
    data_handling::WriteTrajectoriesToFile(traj, file_traj);

    std::cout << "Writing local Flow Field to file." << std::endl;
    Eigen::MatrixXf Vx, Vy;
    cf::GridSize gs;
    ff_cf.GenerateLocalFlowField(gs, &Vx, &Vy);
    data_handling::WriteFlowFieldToFile(Vx, Vy, file_field);
  }
  file_traj_world.close();
  file_traj_local.close();
  file_meta.close();
  if (cp_type == "GOAL")
    file_goal.close();

  return 0;
}
