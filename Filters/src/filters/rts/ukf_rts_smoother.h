#ifndef UKF_RTS_SMOOTHER_H
#define UKF_RTS_SMOOTHER_H

#include "filters/ukf/ukf.h"
#include "measurement_package.h"
#include <vector>

struct UKFRTSStateEstimate {
  long long timestamp = 0;
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
};

class UKFRTSSmoother {
 public:
  std::vector<UKFRTSStateEstimate> Smooth(
      const std::vector<MeasurementPackage>& measurements);

  std::vector<UKFRTSStateEstimate> SmoothFromHistory(
      const std::vector<UKFRTSStepData>& history) const;

 private:
  void NormalizeAngle(double& angle) const;
};

#endif
