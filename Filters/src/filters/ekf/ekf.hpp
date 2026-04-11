#ifndef EKF_H
#define EKF_H

#include "measurement_package.h"
#include <Eigen/Dense>
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// Forward-pass snapshot needed by the EKF-based RTS and fixed-lag smoothers.
struct EKFRTSStepData {
    long long timestamp = 0;
    bool is_initialization = false;
    VectorXd x_filtered;
    MatrixXd P_filtered;
    VectorXd x_predicted;
    MatrixXd P_predicted;
    MatrixXd F_jacobian;
};

class EKF{
    public:
        // Constructor - Initializes EKF with default parameters
        EKF();

        // Destructor
        virtual ~EKF();

        // Process a measurement (lidar or radar) - main entry point for filter update
        void ProcessMeasurument(MeasurementPackage meas_package);

        // Predict state and covariance forward in time using CTRV motion model
        void Prediction(double delta_t);

        // State vector: [px, py, v, yaw, yaw_rate] under a CTRV motion model.
        VectorXd x_;
        
        // State covariance matrix
        MatrixXd P_;

        // Clear cached forward-pass history used by offline smoothers.
        void ClearStepHistory();

        // Access cached forward-pass history for RTS smoothing.
        const std::vector<EKFRTSStepData>& GetStepHistory() const;

    private:
        // Update state using lidar measurement (linear measurement model)
        void UpdateLidar(const MeasurementPackage& meas_package);
        
        // Update state using radar measurement (nonlinear measurement model with Jacobian)
        void UpdateRadar(const MeasurementPackage& meas_package);
        
        // Calculate Jacobian matrix for radar measurement (linearization of h(x))
        Eigen::MatrixXd CalculateRadarJacobian(const Eigen::VectorXd& x_state);
        
        // Normalize angle to [-pi, pi] range
        void NormalizeAngle(double& angle);

        // State transition matrix F for prediction
        MatrixXd F_;      
        
        // Lidar measurement matrix (maps state to measurement space)
        MatrixXd H_laser_;
        
        // Lidar measurement noise covariance matrix
        MatrixXd R_laser_;
        
        // Radar measurement Jacobian (linearized measurement function)
        MatrixXd H_radar_;
        
        // Radar measurement noise covariance matrix
        MatrixXd R_radar_;
        
        // Process noise covariance matrix
        MatrixXd Q_;
        
        // Flag to check if filter has been initialized
        bool _is_initilized;
        
        // Flag to enable/disable lidar measurements
        bool use_laser_;
        
        // Flag to enable/disable radar measurements
        bool use_radar_;
        
        // Timestamp of previous measurement (in microseconds)
        long long time_us_;

        // Process noise standard deviation longitudinal acceleration in m/s^2

        double std_a_;

        // Process noise standard deviation yaw acceleration in rad/s^2
        double std_yawdd_;

        // Laser measurement noise standard deviation position1 in m
        double std_laspx_;

        // Laser measurement noise standard deviation position2 in m
        double std_laspy_;

        // Radar measurement noise standard deviation radius in m
        double std_radr_;

        // Radar measurement noise standard deviation angle in rad
        double std_radphi_;

        // Radar measurement noise standard deviation radius change in m/s
        double std_radrd_ ;

        // Last one-step prediction cached so backward smoothers can reconstruct
        // the RTS gain without rerunning the forward pass.
        VectorXd last_x_pred_;
        MatrixXd last_P_pred_;
        MatrixXd last_F_jacobian_;

        // Forward-pass history for offline RTS smoothing.
        std::vector<EKFRTSStepData> step_history_;

};

#endif
