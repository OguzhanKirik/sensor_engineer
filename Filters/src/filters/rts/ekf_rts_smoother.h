#ifndef EKF_RTS_SMOOTHER_H
#define EKF_RTS_SMOOTHER_H

#include "filters/ekf/ekf.hpp"
#include "measurement_package.h"
#include <vector>

// Smoothed state returned by the backward RTS recursion.
struct RTSStateEstimate {
    long long timestamp = 0;
    VectorXd x;
    MatrixXd P;
};

class EKFRTSSmoother {
  public:
    // Runs a forward EKF pass and then applies the Rauch-Tung-Striebel backward pass.
    std::vector<RTSStateEstimate> Smooth(const std::vector<MeasurementPackage>& measurements);

    // Apply the RTS backward pass to an existing EKF forward-pass history.
    // This is the path used by the highway harness after a simulation run.
    std::vector<RTSStateEstimate> SmoothFromHistory(const std::vector<EKFRTSStepData>& history) const;

  private:
    // Normalizes yaw-related residuals during the backward correction.
    void NormalizeAngle(double& angle) const;
};

#endif
