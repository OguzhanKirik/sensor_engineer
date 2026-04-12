#ifndef UKF_RTS_SMOOTHER_H
#define UKF_RTS_SMOOTHER_H

#include "filters/ukf/ukf.h"
#include "measurement_package.h"
#include <vector>

// Smoothed state returned by the backward unscented RTS recursion.
struct UKFRTSStateEstimate {
  long long timestamp = 0;
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
};

class UKFRTSSmoother {
 public:
  // Convenience path that reruns a fresh UKF forward pass before smoothing.
  std::vector<UKFRTSStateEstimate> Smooth(
      const std::vector<MeasurementPackage>& measurements);

  // Backward unscented RTS pass using sigma-point cross-covariances captured
  // during the original UKF forward pass.
  std::vector<UKFRTSStateEstimate> SmoothFromHistory(
      const std::vector<UKFRTSStepData>& history) const;

 private:
  // Keeps yaw residuals consistent near the +/-pi wrap boundary.
  void NormalizeAngle(double& angle) const;
};

#endif
