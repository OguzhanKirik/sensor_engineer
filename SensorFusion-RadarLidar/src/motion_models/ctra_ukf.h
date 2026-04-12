#ifndef CTRA_UKF_H
#define CTRA_UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Constant Turn Rate and Acceleration UKF for IMM
 * State: [px, py, v, yaw, yaw_rate, longitudinal_acc]
 */
class CTRA_UKF {
public:
    CTRA_UKF();
    virtual ~CTRA_UKF();

    void Initialize(const VectorXd& x_in, const MatrixXd& P_in);
    void Predict(double dt);
    void Update(const MeasurementPackage& meas_package);
    double CalculateLikelihood(const MeasurementPackage& meas_package);

    // State vector [px, py, v, yaw, yaw_rate, a]
    VectorXd x_;
    
    // State covariance matrix
    MatrixXd P_;

    // Get state in 5D format [px, py, v, yaw, yaw_rate] for IMM fusion
    VectorXd GetState5D() const;
    
    // Set state from 5D format
    void SetStateFrom5D(const VectorXd& x_5d);

private:
    void UpdateLidar(const MeasurementPackage& meas_package);
    void UpdateRadar(const MeasurementPackage& meas_package);
    void PredictSigmaPoints(double dt);
    void PredictMeanAndCovariance();
    
    // UKF parameters
    int n_x_;           // State dimension (6)
    int n_aug_;         // Augmented state dimension (8)
    double lambda_;     // Sigma point spreading parameter
    
    VectorXd weights_;  // Sigma point weights
    MatrixXd Xsig_pred_; // Predicted sigma points
    
    // Measurement noise
    double std_laspx_;
    double std_laspy_;
    double std_radr_;
    double std_radphi_;
    double std_radrd_;
    
    // Process noise
    double std_a_;      // Longitudinal acceleration noise
    double std_yawdd_;  // Yaw acceleration noise
    
    bool is_initialized_;
};

#endif // CTRA_UKF_H
