#ifndef EKF_FIXED_LAG_SMOOTHER_H
#define EKF_FIXED_LAG_SMOOTHER_H

#include "filters/rts/ekf_rts_smoother.h"

class EKFFixedLagSmoother {
 public:
  // Smooth each time index using only a bounded future window [k, k + lag].
  std::vector<RTSStateEstimate> SmoothFromHistory(
      const std::vector<EKFRTSStepData>& history, int lag_steps) const;

 private:
  // Keeps yaw residuals consistent during the local backward pass.
  void NormalizeAngle(double& angle) const;
};

#endif
