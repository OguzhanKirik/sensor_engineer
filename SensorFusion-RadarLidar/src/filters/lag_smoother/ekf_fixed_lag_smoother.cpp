#include "filters/lag_smoother/ekf_fixed_lag_smoother.h"

#include <algorithm>

// Fixed-lag variant of the EKF RTS smoother. Each state is corrected with only
// a bounded amount of future information.

std::vector<RTSStateEstimate> EKFFixedLagSmoother::SmoothFromHistory(
    const std::vector<EKFRTSStepData>& history, int lag_steps) const {
  std::vector<RTSStateEstimate> output;
  if (history.empty()) {
    return output;
  }

  const int effective_lag = std::max(1, lag_steps);
  output.resize(history.size());

  for (size_t target = 0; target < history.size(); ++target) {
    const int end =
        std::min<int>(static_cast<int>(history.size()) - 1,
                      static_cast<int>(target) + effective_lag);

    // Recreate a local filtered trajectory and only run the backward pass
    // inside the chosen lag window.
    std::vector<RTSStateEstimate> local(end + 1);
    for (int i = 0; i <= end; ++i) {
      local[i].timestamp = history[i].timestamp;
      local[i].x = history[i].x_filtered;
      local[i].P = history[i].P_filtered;
    }

    for (int k = end - 1; k >= static_cast<int>(target); --k) {
      if (history[k + 1].is_initialization) {
        continue;
      }

      const MatrixXd& P_filtered = history[k].P_filtered;
      const MatrixXd& F_jacobian = history[k + 1].F_jacobian;
      const MatrixXd& P_predicted = history[k + 1].P_predicted;
      MatrixXd smoother_gain =
          P_filtered * F_jacobian.transpose() * P_predicted.inverse();

      VectorXd state_residual = local[k + 1].x - history[k + 1].x_predicted;
      NormalizeAngle(state_residual(3));

      local[k].x = history[k].x_filtered + smoother_gain * state_residual;
      NormalizeAngle(local[k].x(3));

      local[k].P = history[k].P_filtered +
                   smoother_gain * (local[k + 1].P - P_predicted) *
                       smoother_gain.transpose();
    }

    output[target] = local[target];
  }

  return output;
}

void EKFFixedLagSmoother::NormalizeAngle(double& angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}
