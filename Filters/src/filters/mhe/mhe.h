#ifndef MHE_H
#define MHE_H

#include <deque>
#include <vector>

#include <Eigen/Dense>

#include "../../measurement_package.h"
#include "../ekf/ekf.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct MHENode {
  long long timestamp = 0;
  MeasurementPackage measurement;
  VectorXd x_warm_start;
  MatrixXd P_warm_start;
};

class MHE {
 public:
  MHE();
  virtual ~MHE();

  // Process the latest measurement and refresh the moving horizon window.
  void ProcessMeasurement(const MeasurementPackage& meas_package);

  // Clear all internal state and restart the estimator.
  void Reset();

  // Configure the number of measurement nodes retained in the horizon.
  void SetWindowSize(size_t window_size);

  // Set the arrival state/covariance used to anchor the horizon.
  void SetArrivalCost(const VectorXd& x_arrival, const MatrixXd& P_arrival);

  // Solve the current horizon problem.
  // Current implementation returns the EKF warm-start estimate until a nonlinear optimizer is added.
  void Solve();

  // Current estimate [px, py, v, yaw, yaw_rate].
  VectorXd x_;

  // Estimate covariance associated with x_.
  MatrixXd P_;

  // Access the retained horizon for debugging and future solver work.
  const std::deque<MHENode>& GetWindow() const;

  // Returns true once at least one measurement has initialized the estimator.
  bool is_initialized_;

 private:
  void TrimWindow();

  size_t window_size_;
  long long time_us_;

  VectorXd x_arrival_;
  MatrixXd P_arrival_;

  EKF warm_start_ekf_;
  std::deque<MHENode> window_;
};

#endif
