#include "filters/rts/ukf_rts_smoother.h"

std::vector<UKFRTSStateEstimate> UKFRTSSmoother::Smooth(
    const std::vector<MeasurementPackage>& measurements) {
  UKF ukf;
  ukf.ClearStepHistory();

  for (const auto& measurement : measurements) {
    ukf.ProcessMeasurement(measurement);
  }

  return SmoothFromHistory(ukf.GetStepHistory());
}

std::vector<UKFRTSStateEstimate> UKFRTSSmoother::SmoothFromHistory(
    const std::vector<UKFRTSStepData>& history) const {
  std::vector<UKFRTSStateEstimate> smoothed_states;
  if (history.empty()) {
    return smoothed_states;
  }

  smoothed_states.resize(history.size());
  for (size_t i = 0; i < history.size(); ++i) {
    smoothed_states[i].timestamp = history[i].timestamp;
    smoothed_states[i].x = history[i].x_filtered;
    smoothed_states[i].P = history[i].P_filtered;
  }

  for (int k = static_cast<int>(history.size()) - 2; k >= 0; --k) {
    if (history[k + 1].is_initialization) {
      continue;
    }

    const Eigen::MatrixXd& P_cross = history[k + 1].P_cross;
    const Eigen::MatrixXd& P_predicted = history[k + 1].P_predicted;
    Eigen::MatrixXd smoother_gain = P_cross * P_predicted.inverse();

    Eigen::VectorXd state_residual =
        smoothed_states[k + 1].x - history[k + 1].x_predicted;
    NormalizeAngle(state_residual(3));

    smoothed_states[k].x =
        history[k].x_filtered + smoother_gain * state_residual;
    NormalizeAngle(smoothed_states[k].x(3));

    smoothed_states[k].P =
        history[k].P_filtered +
        smoother_gain * (smoothed_states[k + 1].P - P_predicted) *
            smoother_gain.transpose();
  }

  return smoothed_states;
}

void UKFRTSSmoother::NormalizeAngle(double& angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}
