#ifndef UKF_FIXED_LAG_SMOOTHER_H
#define UKF_FIXED_LAG_SMOOTHER_H

#include "filters/rts/ukf_rts_smoother.h"

class UKFFixedLagSmoother {
 public:
  // Smooth each time index using only a bounded future window [k, k + lag].
  std::vector<UKFRTSStateEstimate> SmoothFromHistory(
      const std::vector<UKFRTSStepData>& history, int lag_steps) const;

 private:
  // Keeps yaw residuals consistent during the local backward pass.
  void NormalizeAngle(double& angle) const;
};

#endif
