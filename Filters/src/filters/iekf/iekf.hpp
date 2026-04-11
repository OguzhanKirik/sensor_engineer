#ifndef IEKF_H
#define IEKF_H

#include "measurement_package.h"
#include <Eigen/Dense>
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;


class IEKF{
    public:
        // Constructor - Initializes IEKF with default parameters
        IEKF();

        // Destructor
        virtual ~IEKF();

        // Process a measurement (lidar or radar) - main entry point for filter update
        void ProcessMeasurument(MeasurementPackage meas_package);

        // Predict state and covariance forward in time using CTRV motion model
        void Prediction(double delta_t);

        // State vector: [px, py, v, yaw, yaw_rate]
        VectorXd x_;
        
        // State covariance matrix
        MatrixXd P_;

    private:
        // Update state using lidar measurement (linear measurement model)
        void UpdateLidar(const MeasurementPackage& meas_package);
        
        // Update state using radar measurement (nonlinear - uses iterative refinement)
        void UpdateRadar(const MeasurementPackage& meas_package);
        
        // Iterative update helper for radar measurements
        void UpdateRadarIterative(const Eigen::VectorXd& z);
        
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
        double std_radrd_;
        
        // ==========================================
        // IEKF-Specific Parameters
        // ==========================================
        
        // Number of iterations for measurement update
        int max_iterations_;
        
        // Convergence threshold for iterative update
        double convergence_threshold_;

};

#endif
