#ifndef IMM_HPP
#define IMM_HPP

/*
IMM (Interacting Multiple Model) Filter
Combines four motion models:
- CV: Constant Velocity (straight motion, no acceleration)
- CA: Constant Acceleration (accelerating/decelerating)
- CTRV: Constant Turn Rate and Velocity (turning at steady speed)
- CTRA: Constant Turn Rate and Acceleration (turning while accelerating)
*/

#include "Eigen/Dense"
#include "../../measurement_package.h"
#include "../../motion_models/cv_ekf.h"
#include "../../motion_models/ca_ekf.h"
#include "../../motion_models/ctrv_ukf.h"
#include "../../motion_models/ctra_ukf.h"
#include <array>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class IMM {
public:
    IMM();
    virtual ~IMM();
    
    // Main processing function
    void ProcessMeasurement(const MeasurementPackage& meas_package);
    
    // Get fused state and covariance
    VectorXd GetState() const { return x_; }
    MatrixXd GetCovariance() const { return P_; }
    VectorXd GetModelProbabilities() const { return mu_; }

private:
    static const int N_MODELS = 4;  // CV, CA, CTRV, CTRA
    static const int N_X = 5;       // State dimension [px, py, v, yaw, yaw_rate]
    
    // Model indices
    enum ModelIndex {
        CV = 0,    // Constant Velocity
        CA = 1,    // Constant Acceleration
        CTRV = 2,  // Constant Turn Rate and Velocity
        CTRA = 3   // Constant Turn Rate and Acceleration
    };

    // Individual filter instances
    CV_EKF cv_filter_;
    CA_EKF ca_filter_;
    CTRV_UKF ctrv_filter_;
    CTRA_UKF ctra_filter_;
    
    // Model states and covariances (after mixing, before update)
    std::array<VectorXd, N_MODELS> x_mixed_;
    std::array<MatrixXd, N_MODELS> P_mixed_;
    
    // Model probabilities
    VectorXd mu_;          // Current model probabilities [mu_CV, mu_CTRV]
    
    // Mode transition probability matrix
    // PI_(i,j) = probability of switching from model i to model j
    MatrixXd PI_;
    
    // Mixing probabilities
    MatrixXd mu_ij_;  // mu_ij_(i,j) = P(model i at k-1 | model j at k | Z)
    VectorXd c_j_;    // Normalization constants

    // Fused state and covariance (output of IMM)
    VectorXd x_;
    MatrixXd P_;

    // Initialization and timing
    bool is_initialized_;
    long long previous_timestamp_;

    // IMM algorithm steps
    void MixStates();
    void Predict(double dt);
    void Update(const MeasurementPackage& meas_package);
    void UpdateModelProbabilities(const std::array<double, N_MODELS>& likelihoods);
    void FuseEstimate();
    
    // Helper functions
    void Initialize(const MeasurementPackage& meas_package);
    void NormalizeAngle(double& angle);
};

#endif // IMM_HPP