#ifndef CKF_H
#define CKF_H

#include "Eigen/Dense"
#include "../../measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class CKF {
public:
    /**
     * Constructor
     */
    CKF();

    /**
     * Destructor
     */
    virtual ~CKF();

    /**
     * Initialize the CKF with first measurement
     * @param meas_package The measurement package
     */
    void Initialize(const VectorXd& x, const MatrixXd& P);

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(const MeasurementPackage& meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Predict(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(const MeasurementPackage& meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(const MeasurementPackage& meas_package);

    /**
     * Calculate likelihood for IMM
     */
    double CalculateLikelihood(const MeasurementPackage& meas_package);

    // Current state vector [px, py, v, yaw, yaw_rate]
    VectorXd x_;

    // State covariance matrix
    MatrixXd P_;

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

    // Weights for cubature points. CKF uses equal weights by construction.
    VectorXd weights_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Number of cubature points (2 * n_x for CKF)
    int n_sigma_;

    // Predicted cubature points after process-model propagation.
    MatrixXd Xsig_pred_;

    // Time when the state is true, in us
    long long time_us_;

    // Initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

private:
    /**
     * Generate cubature points
     */
    MatrixXd GenerateCubaturePoints(const VectorXd& x, const MatrixXd& P);

    /**
     * Predict cubature points through process model
     */
    void PredictCubaturePoints(const MatrixXd& Xsig, double delta_t);

    /**
     * CTRV process model
     */
    VectorXd ProcessModel(const VectorXd& x, double delta_t, double nu_a, double nu_yawdd);

    /**
     * Normalize angle to [-pi, pi]
     */
    void NormalizeAngle(double& angle);
};

#endif  // CKF_H
