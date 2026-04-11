#ifndef UKF_FIXED_LAG_SMOOTHER_H
#define UKF_FIXED_LAG_SMOOTHER_H

#include "filters/rts/ukf_rts_smoother.h"

class UKFFixedLagSmoother {
 public:
  std::vector<UKFRTSStateEstimate> SmoothFromHistory(
      const std::vector<UKFRTSStepData>& history, int lag_steps) const;

 private:
  void NormalizeAngle(double& angle) const;
};

#endif
