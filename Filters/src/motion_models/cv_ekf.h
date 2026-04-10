#ifndef CV_EKF_H
#define CV_EKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Simple Constant Velocity EKF for IMM
 * State: [px, py, vx, vy]
 */
class CV_EKF {
public:
    CV_EKF();
    virtual ~CV_EKF();

    void Initialize(const VectorXd& x_in, const MatrixXd& P_in);
    void Predict(double dt);
    void Update(const MeasurementPackage& meas_package);
    double CalculateLikelihood(const MeasurementPackage& meas_package);

    // State vector [px, py, vx, vy]
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
    
    MatrixXd H_laser_;   // Lidar measurement matrix
    MatrixXd R_laser_;   // Lidar measurement noise
    MatrixXd R_radar_;   // Radar measurement noise
    
    double std_a_;       // Process noise acceleration
    double std_laspx_;
    double std_laspy_;
    double std_radr_;
    double std_radphi_;
    double std_radrd_;
    
    bool is_initialized_;
};

#endif // CV_EKF_H
