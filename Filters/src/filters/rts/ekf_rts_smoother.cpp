#include "filters/rts/ekf_rts_smoother.h"

#include <stdexcept>

std::vector<RTSStateEstimate> EKFRTSSmoother::Smooth(
    const std::vector<MeasurementPackage>& measurements) {
  EKF ekf;
  ekf.ClearStepHistory();

  for (const auto& measurement : measurements) {
    ekf.ProcessMeasurument(measurement);
  }

  return SmoothFromHistory(ekf.GetStepHistory());
}

std::vector<RTSStateEstimate> EKFRTSSmoother::SmoothFromHistory(
    const std::vector<EKFRTSStepData>& history) const {
  std::vector<RTSStateEstimate> smoothed_states;
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

    const MatrixXd& P_filtered = history[k].P_filtered;
    const MatrixXd& F_jacobian = history[k + 1].F_jacobian;
    const MatrixXd& P_predicted = history[k + 1].P_predicted;

    MatrixXd smoother_gain =
        P_filtered * F_jacobian.transpose() * P_predicted.inverse();

    VectorXd state_residual =
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

void EKFRTSSmoother::NormalizeAngle(double& angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}
