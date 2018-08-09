#pragma once

#include <deque>
#include <vector>

#include <libgp_interface.h>

#include <cost_function/cost_function.h>
#include <trajectory/state.h>
#include <trajectory/trajectory.h>

namespace cost_function {

// Define grid size for the local flow field, with some default params.
struct GridSize {
  size_t Nx = 51;
  size_t Ny = 51;
  float min_x = 0;
  float max_x = 30;
  float min_y = -15;
  float max_y = 15;
};

// Define parameters to Gaussian Processes that will be used for the flow field.
// TODO(xuning@cmu.edu): Add default parameter and parameter handling.
struct GP {
  unsigned int dim = 0;
  std::string cov_kernel = "CovSum( CovSEiso, CovNoise)";
  std::vector<double> hyp_params_x;
  std::vector<double> hyp_params_y;
};

// `FlowField` is an inherited class that learns and computes cost based on a
// local field of directional intent.
class FlowField : public CostFunction {
public:
  FlowField();
  ~FlowField();

  // Initializes the two Gaussian Processes that learns the x and y velocity.
  void Initialize(const GP &gp, const size_t window_size);

  // Add a segment of trajectory to the set of training examples to be learned.
  void AddSegment(const std::vector<trajectory::State<float>> &seg);

  // Implementation of the virtual method from the parent class.
  std::vector<float>
  ComputeCost(std::vector<trajectory::Trajectory<float>> &traj) override;

  // A variation of the ComputeCost function, given waypoints position `pos` and
  // velocity `vel` each in a 3xN matrix, where N is the number of waypoints.
  float ComputeCost(const Eigen::Matrix<float, 3, Eigen::Dynamic> &pos,
                    const Eigen::Matrix<float, 3, Eigen::Dynamic> &vel);

  // Clears training examples in the two GPs.
  void Clear();

  // Adds training examples to the GPs and update the GPs.
  void Learn();

  // Given a specific grid, output the predicted X-Y velocities over this field.
  void GenerateLocalFlowField(const GridSize &gs, Eigen::MatrixXf *Vx,
                              Eigen::MatrixXf *Vy);

private:
  // GPs representing the X and Y velocities.
  LibgpInterface gp_x_;
  LibgpInterface gp_y_;

  // Set of training examples.
  std::deque<trajectory::State<float>> training_examples_;

  // A deque that contains the number of training examples added per window.
  std::deque<size_t> num_examples_;

  // The number of past trajectory segments used to learn the GP.
  size_t window_size_;
};

} // namespace cost_function
