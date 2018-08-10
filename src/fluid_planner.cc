#include <fluid_planner.h>

#include <chrono>
#include <iostream>

namespace tr = trajectory;
namespace cf = cost_function;
namespace cu = conversion_utils;
namespace su = stats_utils;
namespace lu = linalg_utils;
namespace vu = vector_utils;

namespace fluid_planner {

FluidPlanner::FluidPlanner() {}
FluidPlanner::~FluidPlanner() {}

void FluidPlanner::Initialize(const PlannerProperties &pp,
                              const TrajProperties &tp) {
  SetPlannerProperties(pp);
  SetTrajectoryProperties(tp);

  // Initialize matrix added to covariance matrices to prevent degeneracy.
  pertubation_ = Eigen::MatrixXf::Identity(tp_.num_seg * 2, tp_.num_seg * 2) *
                 pp_.sample_dev;

  // Sample trajectories.
  traj_ = SampleTrajectoriesRandom(pp.num_traj);

  // Turn trajectory Z representation into a matrix. Each column represents a
  // trajectory.
  Eigen::MatrixXf samples(tp_.num_seg * 2, pp_.num_traj);
  int i = 0;
  for (auto &tr : traj_) {
    samples.col(i) = tr.Z();
    i++;
  }

  // Initialize GMM.
  Gmm gmm;
  gmm.k = pp_.k;
  std::vector<tr::Trajectory<float>> traj_rand =
      SampleTrajectoriesRandom(gmm.k);
  const float weight = 1.0 / traj_rand.size();
  for (size_t i = 0; i < pp_.k; i++) {
    gmm.mean.push_back(traj_rand[i].Z());
    auto cov = su::Covariance<float>(samples, traj_rand[i].Z());
    gmm.cov.push_back(cov + pertubation_);
    gmm.cov_sqrt.push_back((cov + pertubation_).sqrt());
    gmm.weights.push_back(weight);
  }
  gmm_ = UpdateGmm(traj_, gmm);
}

Gmm FluidPlanner::UpdateGmm(const std::vector<tr::Trajectory<float>> &traj,
                            const Gmm &gmm) {

  // Turn trajectory Z representation into a matrix. Each column represents a
  // trajectory.
  Eigen::MatrixXf samples(tp_.num_seg * 2, traj.size());
  int i = 0;
  for (auto &tr : traj) {
    samples.col(i) = tr.Z();
    i++;
  }

  const int K = pp_.k;
  const size_t N = traj.size();

  Gmm updated_gmm;
  if (K == 1) {
    // Compute mean and covariance.
    Eigen::VectorXf mean = samples.rowwise().mean();
    Eigen::MatrixXf centered = samples.colwise() - mean;
    Eigen::MatrixXf cov = (centered * centered.transpose()) / N;

    updated_gmm.k = 1;
    updated_gmm.weights = std::vector<float>(K, 1.0 / float(K));
    updated_gmm.mean.push_back(mean);
    updated_gmm.cov.push_back(cov + pertubation_);
    updated_gmm.cov_sqrt.push_back((cov + pertubation_).sqrt());

  } else if (K > 1) {

    updated_gmm.k = K;

    // Determine membership.
    Eigen::MatrixXf W(N, K);
    for (size_t i = 0; i < N; i++) {
      Eigen::VectorXf p(K);
      auto tj = traj[i];
      for (int k = 0; k < K; k++) {
        float p1 = su::GaussianPdf<float>(tj.Z(), gmm.mean[k], gmm.cov[k]);
        p(k) = su::GaussianPdf<float>(tj.Z(), gmm.mean[k], gmm.cov[k]) *
               gmm.weights[k];
      }
      W.row(i) = p.array() / p.sum();
    }

    // Update mean and covariance.
    for (int k = 0; k < K; k++) {
      float w_total = W.col(k).sum();
      Eigen::VectorXf mean = samples * W.col(k) / w_total;
      Eigen::MatrixXf cov = su::Covariance<float>(samples, mean, W.col(k));

      updated_gmm.weights.push_back(w_total / N);
      updated_gmm.mean.push_back(mean);
      updated_gmm.cov.push_back(cov + pertubation_);
      updated_gmm.cov_sqrt.push_back((cov + pertubation_).sqrt());
    }
  }

  return updated_gmm;
}

std::vector<tr::Trajectory<float>>
FluidPlanner::Plan(const tr::State<float> &start, cf::CostFunction &cost_fcn) {
  auto t0 = std::chrono::high_resolution_clock::now();
  int num_elite;
  float rho_quantile_cost = std::numeric_limits<float>::max();
  std::vector<tr::Trajectory<float>> elite_traj;

  // Go through num_iter iterations of planning.
  for (int iter = 0; iter < pp_.num_iter; iter++) {
    printf("Iter: %d ", iter);
    std::cout << std::flush;
    elite_traj.clear();

    auto t1 = std::chrono::high_resolution_clock::now();

    // Sample trajectories.
    printf(" Sampling trajs");
    std::cout << std::flush;
    std::vector<tr::Trajectory<float>> sampled_traj;
    if (iter == 0)
      sampled_traj = traj_;
    else
      sampled_traj = SampleTrajectoriesGMM(gmm_, pp_.num_traj);

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> dur = (t2 - t1);
    printf("(%.4fs)\t", dur.count());

    // Set transforms for the trajectories.
    printf("Set Tsfms");
    std::cout << std::flush;
    Eigen::Transform<float, 3, 3> R_wb;
    R_wb = Eigen::AngleAxisf(start.Yaw(), Eigen::Vector3f::UnitZ());
    Eigen::Vector3f T_wb = start.Pos();
    for (auto &tr : sampled_traj) {
      tr.SetTransform(R_wb, T_wb);
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    dur = (t3 - t2);
    printf("(%.4fs)\t", dur.count());

    // Compute objective for all trajectories.
    printf("Compute cost");
    std::cout << std::flush;
    std::vector<float> cost = cost_fcn.ComputeCost(sampled_traj);
    auto t4 = std::chrono::high_resolution_clock::now();

    dur = (t4 - t3);
    printf("(%.4fs)\t", dur.count());
    std::cout << std::flush;

    // Obtain trajectories that are below the rho-quantile cost.
    // Sort cost in ascending order.
    printf("Sorting cost");
    std::cout << std::flush;
    std::vector<float> cost_sorted;
    std::vector<size_t> cost_sorted_idx;
    vu::Sort<float>(cost, &cost_sorted, &cost_sorted_idx);

    auto t5 = std::chrono::high_resolution_clock::now();
    dur = (t5 - t4);
    printf("(%.4fs)\t", dur.count());
    std::cout << std::flush;

    printf("Elite set");
    std::cout << std::flush;
    // Enforce to select at least 2 elite trajectories.
    num_elite = 0;
    while (num_elite < 2) {
      // Find (index of the maximum lower bound) + 1.
      std::vector<float>::iterator it = std::lower_bound(
          cost_sorted.begin(), cost_sorted.end(), rho_quantile_cost);

      // Get corresponding index of the maximum lower bound.
      num_elite = (it == cost_sorted.end()) ? cost_sorted.size()
                                            : it - cost_sorted.begin();

      // If the number of trajectories are less than 2, increase the cost by 5%
      // and try it again.
      if (num_elite < 2) {
        rho_quantile_cost += std::abs(0.05 * rho_quantile_cost);
      }
    }
    printf(", found %d", num_elite);
    auto t6 = std::chrono::high_resolution_clock::now();

    dur = (t6 - t5);
    printf("(%.4fs)\t", dur.count());
    std::cout << std::flush;

    // Update Rho quantile cost.
    if (iter == 0) {
      rho_quantile_cost =
          cost_sorted[std::ceil(pp_.rho * (cost_sorted.size() - 1))];
      float cost = cost_sorted[0];
      size_t i = 0;
      while (cost < rho_quantile_cost) {
        elite_traj.push_back(sampled_traj[cost_sorted_idx[i]]);
        i++;
        cost = cost_sorted[i];
      }
    } else {
      // Collect all the elite trajectories.
      for (size_t i = 0; i < num_elite; i++) {
        elite_traj.push_back(sampled_traj[cost_sorted_idx[i]]);
      }
      // Update rho-quantile cost.
      printf("Update quantile");
      rho_quantile_cost =
          cost_sorted[std::ceil(pp_.rho * (cost_sorted.size() - 1))];
    }
    std::cout << "rho_quantile_cost " << rho_quantile_cost << std::endl;
    auto t7 = std::chrono::high_resolution_clock::now();
    dur = (t7 - t6);
    printf("(%.4fs)\t", dur.count());

    // Update GMMs.
    printf("Update GMMs");
    std::cout << std::flush;
    gmm_ = UpdateGmm(elite_traj, gmm_);

    // Output timing information.
    auto t8 = std::chrono::high_resolution_clock::now();
    dur = (t8 - t7);
    printf("(%.4fs)\t", dur.count());
    dur = (t8 - t1);
    printf("TOTAL(%.4fs)\n", dur.count());
  }

  traj_ = elite_traj;
  auto tf = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> dur = (tf - t0);
  printf("Planner finished, took %.4fs. Exiting.\n", dur.count());
  return traj_;
}

void FluidPlanner::SetPlannerProperties(const PlannerProperties &pp) {
  pp_ = pp;
}

void FluidPlanner::SetTrajectoryProperties(const TrajProperties &tp) {
  assert(tp.traj_duration > 0);
  assert(tp.num_seg > 0);

  tp_ = tp;
  seg_duration_ = tp.traj_duration / tp.num_seg;
  SetInputRange(tp.v_lb, tp.v_ub, tp.omega_lb, tp.omega_ub,
                tp.num_action_space_discretizations);
}

void FluidPlanner::SetInputRange(float v_lb, float v_ub, float omega_lb,
                                 float omega_ub, int N) {
  assert(N > 1);

  vel_vector_ = lu::Linspace<float>(v_lb, v_ub, N);
  omega_vector_ = lu::Linspace<float>(omega_lb, omega_ub, N);
}

std::vector<tr::Trajectory<float>>
FluidPlanner::SampleTrajectoriesRandom(const int num_traj) {
  // Check the input is valid.
  assert(num_traj > 0);

  // Generate trajectories.
  std::vector<tr::Trajectory<float>> trajectories;
  for (int i = 0; i < num_traj; i++) {
    std::vector<float> omegas = su::DataSample(omega_vector_, tp_.num_seg);
    std::vector<float> vels = su::DataSample(vel_vector_, tp_.num_seg);
    tr::Trajectory<float> traj(vels, omegas, seg_duration_);
    trajectories.push_back(traj);
  }
  return trajectories;
}

std::vector<tr::Trajectory<float>>
FluidPlanner::SampleTrajectoriesGMM(const Gmm &gmm, const int num_traj) {
  // Check the inputs are valid.
  assert(num_traj > 0);
  assert(gmm.k > 0);
  assert(gmm.k == gmm.weights.size());

  // Sample from the trajectory GMM if there is more than one mixture.
  std::vector<int> k_vec;
  if (gmm.k > 1)
    k_vec = su::DiscreteSample(gmm.weights, num_traj);

  // Generate trajectories.
  std::vector<tr::Trajectory<float>> trajectories;
  for (int i = 0; i < num_traj; i++) {
    int k = (gmm.k == 1) ? 0 : k_vec[i];
    Eigen::VectorXf dev =
        su::RangeSample(-pp_.sample_dev, pp_.sample_dev, 2 * tp_.num_seg);
    Eigen::VectorXf Z = gmm.mean[k] + gmm.cov_sqrt[k] * dev;
    tr::Trajectory<float> traj(Z, seg_duration_);
    trajectories.push_back(traj);
  }

  return trajectories;
}

std::vector<tr::Trajectory<float>> FluidPlanner::SampleTrajectoriesGMMRandom(
    const Gmm &gmm, const int num_traj_random, const int num_traj_gmm) {
  // Check the inputs are valid.
  assert(num_traj_random > 0);
  assert(num_traj_gmm > 0);
  assert(gmm.k > 0);

  std::vector<tr::Trajectory<float>> trajectories_gmm =
      SampleTrajectoriesGMM(gmm, num_traj_gmm);
  std::vector<tr::Trajectory<float>> trajectories_rand =
      SampleTrajectoriesRandom(num_traj_random);

  std::vector<tr::Trajectory<float>> trajectories;
  trajectories.reserve(trajectories_gmm.size() + trajectories_rand.size());
  trajectories.insert(trajectories.end(), trajectories_gmm.begin(),
                      trajectories_gmm.end());
  trajectories.insert(trajectories.end(), trajectories_rand.begin(),
                      trajectories_rand.end());

  return trajectories;
}

} // namespace fluid_planner
