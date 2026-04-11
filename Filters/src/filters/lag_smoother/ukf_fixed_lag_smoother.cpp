#include "filters/lag_smoother/ukf_fixed_lag_smoother.h"

#include <algorithm>

std::vector<UKFRTSStateEstimate> UKFFixedLagSmoother::SmoothFromHistory(
    const std::vector<UKFRTSStepData>& history, int lag_steps) const {
  std::vector<UKFRTSStateEstimate> output;
  if (history.empty()) {
    return output;
  }

  const int effective_lag = std::max(1, lag_steps);
  output.resize(history.size());

  for (size_t target = 0; target < history.size(); ++target) {
    const int end =
        std::min<int>(static_cast<int>(history.size()) - 1,
                      static_cast<int>(target) + effective_lag);

    std::vector<UKFRTSStateEstimate> local(end + 1);
    for (int i = 0; i <= end; ++i) {
      local[i].timestamp = history[i].timestamp;
      local[i].x = history[i].x_filtered;
      local[i].P = history[i].P_filtered;
    }

    for (int k = end - 1; k >= static_cast<int>(target); --k) {
      if (history[k + 1].is_initialization) {
        continue;
      }

      const Eigen::MatrixXd& P_cross = history[k + 1].P_cross;
      const Eigen::MatrixXd& P_predicted = history[k + 1].P_predicted;
      Eigen::MatrixXd smoother_gain = P_cross * P_predicted.inverse();

      Eigen::VectorXd state_residual =
          local[k + 1].x - history[k + 1].x_predicted;
      NormalizeAngle(state_residual(3));

      local[k].x = history[k].x_filtered + smoother_gain * state_residual;
      NormalizeAngle(local[k].x(3));

      local[k].P =
          history[k].P_filtered +
          smoother_gain * (local[k + 1].P - P_predicted) *
              smoother_gain.transpose();
    }

    output[target] = local[target];
  }

  return output;
}

void UKFFixedLagSmoother::NormalizeAngle(double& angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}
