#ifndef CTRV_UKF_H
#define CTRV_UKF_H

#include "../filters/ukf/ukf.h"
#include "../measurement_package.h"

/**
 * Wrapper around UKF for IMM usage
 * Uses existing UKF implementation for CTRV model
 */
class CTRV_UKF {
public:
    CTRV_UKF();
    virtual ~CTRV_UKF();

    void Initialize(const Eigen::VectorXd& x_in, const Eigen::MatrixXd& P_in);
    void Predict(double dt);
    void Update(const MeasurementPackage& meas_package);
    double CalculateLikelihood(const MeasurementPackage& meas_package);

    // State vector [px, py, v, yaw, yaw_rate]
    Eigen::VectorXd x_;
    
    // State covariance matrix
    Eigen::MatrixXd P_;

private:
    UKF ukf_;  // Use existing UKF implementation
    bool is_initialized_;
};

#endif // CTRV_UKF_H
