#include <cost_function/flow_field.h>

namespace tr = trajectory;

namespace cost_function {

static constexpr float kZeroSpeed = 0.01;

FlowField::FlowField() {}
FlowField::~FlowField() {}

void FlowField::Initialize(const GP &gp, const size_t window_size) {
  gp_x_.Initialize(gp.dim, gp.cov_kernel, gp.hyp_params_x);
  gp_y_.Initialize(gp.dim, gp.cov_kernel, gp.hyp_params_y);
  window_size_ = window_size;
  return;
}

void FlowField::Learn() {
  for (auto &s : training_examples_) {
    Eigen::Vector2d pos_2d = Eigen::Vector2d(s.Pos(0), s.Pos(1));
    gp_x_.Train(pos_2d, s.Vel(0));
    gp_y_.Train(pos_2d, s.Vel(1));
  }
  return;
}

void FlowField::Clear() {
  gp_x_.Clear();
  gp_y_.Clear();
  return;
}

void FlowField::AddSegment(const std::vector<tr::State<float>> &seg) {
  // If there are too many elements, erase them.
  if (num_examples_.size() >= window_size_) {
    for (size_t i = 0; i < num_examples_.size() - window_size_ + 1; i++) {
      size_t num_examples_to_erase = num_examples_[0];
      num_examples_.pop_front();
      training_examples_.erase(training_examples_.begin(),
                               training_examples_.begin() +
                                   num_examples_to_erase);
    }
  }

  // Add data.
  num_examples_.push_back(seg.size());
  training_examples_.insert(training_examples_.end(), seg.begin(), seg.end());

  // Check that we have the correct number of training sets.
  assert(num_examples_.size() <= window_size_);
  return;
}

std::vector<float>
FlowField::ComputeCost(std::vector<tr::Trajectory<float>> &trajs) {
  std::vector<float> cost;
  for (auto &tj : trajs) {
    Eigen::Matrix<float, 3, Eigen::Dynamic> pos, vel;
    // TODO(xuning@cmu.edu): Make the dt=0.3 a parameter.
    tj.GetWaypointsLocal(0.3, &pos, &vel);
    cost.push_back(ComputeCost(pos, vel));
  }
  return cost;
}

float FlowField::ComputeCost(
    const Eigen::Matrix<float, 3, Eigen::Dynamic> &pos,
    const Eigen::Matrix<float, 3, Eigen::Dynamic> &vel) {

  int num_samples = pos.cols();
  Eigen::VectorXd vel_x, vel_y;
  Eigen::MatrixXd pos_2d = pos.block(0, 0, 2, num_samples).cast<double>();
  gp_x_.Predict(num_samples, pos_2d, &vel_x);
  gp_y_.Predict(num_samples, pos_2d, &vel_y);

  // Reformat matrices.
  Eigen::MatrixXf vel_pred(2, num_samples);
  Eigen::VectorXf vel_x_f = vel_x.cast<float>();
  Eigen::VectorXf vel_y_f = vel_y.cast<float>();
  vel_pred << vel_x_f.transpose(), vel_y_f.transpose();

  Eigen::MatrixXf vel_actual(2, num_samples);
  vel_actual = vel.block(0, 0, 2, num_samples).cast<float>();

  Eigen::VectorXf vel_pred_norm = vel_pred.colwise().norm();
  Eigen::VectorXf vel_actual_norm = vel_actual.colwise().norm();

  // Normalize the vectors.
  vel_pred.colwise().normalize();
  vel_actual.colwise().normalize();

  // The reward is the dot product between the predicted velocity field and the
  // actual velocity vectors.
  Eigen::VectorXf reward = (vel_pred.transpose() * vel_actual).diagonal();

  for (size_t i = 0; i < reward.size(); i++) {
    if (vel_pred_norm(i) < kZeroSpeed) {
      // TODO(xuning@cmu.edu): Make the 'arbitrarily large value for the
      // unpopulated flow field regions' a parameter. Currently set to 10.
      reward(i) = -10;
    }
  }
  Eigen::VectorXf speed_cost = (vel_pred_norm - vel_actual_norm).array().abs();
  Eigen::VectorXf cost = -reward + speed_cost;

  return cost.sum();
}

void FlowField::GenerateLocalFlowField(const GridSize &gs, Eigen::MatrixXf *Vx,
                                       Eigen::MatrixXf *Vy) {
  Eigen::VectorXf pos_x = Eigen::VectorXf::LinSpaced(gs.Nx, gs.min_x, gs.max_x);
  Eigen::VectorXf pos_y = Eigen::VectorXf::LinSpaced(gs.Ny, gs.min_y, gs.max_y);
  Vx->resize(gs.Nx, gs.Ny);
  Vy->resize(gs.Nx, gs.Ny);

  for (size_t i = 0; i < gs.Nx; i++) {
    for (size_t j = 0; j < gs.Ny; j++) {
      Eigen::Vector2d xy_coord(pos_x(i), pos_y(j));
      (*Vx)(i, j) = gp_x_.Predict(xy_coord);
      (*Vy)(i, j) = gp_y_.Predict(xy_coord);
    }
  }
  return;
}

} // namespace cost_function
