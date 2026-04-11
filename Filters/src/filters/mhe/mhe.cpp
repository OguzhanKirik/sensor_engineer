#include "mhe.h"

#include <algorithm>

// Sliding-window MHE scaffold. The nonlinear optimizer is not implemented yet;
// an embedded EKF supplies the warm-start and current output.

MHE::MHE()
    : x_(VectorXd::Zero(5)),
      P_(MatrixXd::Identity(5, 5)),
      is_initialized_(false),
      window_size_(10),
      time_us_(0),
      x_arrival_(VectorXd::Zero(5)),
      P_arrival_(MatrixXd::Identity(5, 5)) {}

MHE::~MHE() {}

void MHE::ProcessMeasurement(const MeasurementPackage& meas_package) {
  // Reuse EKF as a warm-start so the measurement flow and horizon management
  // can be tested before adding a true solver.
  warm_start_ekf_.ProcessMeasurument(meas_package);

  x_ = warm_start_ekf_.x_;
  P_ = warm_start_ekf_.P_;
  time_us_ = meas_package.timestamp_;
  is_initialized_ = true;

  MHENode node;
  node.timestamp = meas_package.timestamp_;
  node.measurement = meas_package;
  node.x_warm_start = x_;
  node.P_warm_start = P_;
  window_.push_back(node);
  TrimWindow();

  // Placeholder: until the nonlinear least-squares backend is implemented,
  // expose the EKF warm-start as the current MHE estimate.
  Solve();
}

void MHE::Reset() {
  x_.setZero();
  P_.setIdentity();
  x_arrival_.setZero();
  P_arrival_.setIdentity();
  time_us_ = 0;
  is_initialized_ = false;
  window_.clear();
  warm_start_ekf_ = EKF();
}

void MHE::SetWindowSize(size_t window_size) {
  window_size_ = std::max<size_t>(1, window_size);
  TrimWindow();
}

void MHE::SetArrivalCost(const VectorXd& x_arrival, const MatrixXd& P_arrival) {
  x_arrival_ = x_arrival;
  P_arrival_ = P_arrival;
}

void MHE::Solve() {
  if (!is_initialized_ || window_.empty()) {
    return;
  }

  // When the true MHE optimizer is added, this method should optimize the full
  // horizon state sequence using x_arrival_/P_arrival_ and window_ residuals.
  x_ = window_.back().x_warm_start;
  P_ = window_.back().P_warm_start;
}

const std::deque<MHENode>& MHE::GetWindow() const { return window_; }

void MHE::TrimWindow() {
  // In a full MHE implementation the dropped node would be summarized into the
  // arrival cost. For now it is simply removed.
  while (window_.size() > window_size_) {
    window_.pop_front();
  }
}
