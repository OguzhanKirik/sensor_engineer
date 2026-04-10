
#include "IMM.hpp"
#include <iostream>
#include <cmath>

// Define static const members
const int IMM::N_MODELS;
const int IMM::N_X;

IMM::IMM() {
    // Initialize state and covariance
    x_ = VectorXd(N_X);
    x_.fill(0.0);
    
    P_ = MatrixXd(N_X, N_X);
    P_ = MatrixXd::Identity(N_X, N_X);
    
    // Initialize model probabilities (equal probability initially)
    mu_ = VectorXd(N_MODELS);
    mu_ << 0.25, 0.25, 0.25, 0.25;  // Equal 25% for each model
    
    // Mode transition probability matrix (4x4)
    // Rows represent "from" model, columns represent "to" model
    // Diagonal elements = probability of staying in same model
    // Off-diagonal = probability of switching models
    PI_ = MatrixXd(N_MODELS, N_MODELS);
    
    // More likely to stay in current model, small chance to switch
    // Structured to favor similar models (CV<->CA, CTRV<->CTRA)
    PI_ << 0.70, 0.15, 0.10, 0.05,  // From CV: prefer CV, then CA
           0.15, 0.70, 0.05, 0.10,  // From CA: prefer CA, then CV
           0.10, 0.05, 0.70, 0.15,  // From CTRV: prefer CTRV, then CTRA
           0.05, 0.10, 0.15, 0.70;  // From CTRA: prefer CTRA, then CTRV
    
    // Initialize mixing probabilities
    mu_ij_ = MatrixXd(N_MODELS, N_MODELS);
    c_j_ = VectorXd(N_MODELS);
    
    // Initialize mixed states
    for (int i = 0; i < N_MODELS; ++i) {
        x_mixed_[i] = VectorXd(N_X);
        x_mixed_[i].fill(0.0);
        P_mixed_[i] = MatrixXd::Identity(N_X, N_X);
    }
    
    is_initialized_ = false;
    previous_timestamp_ = 0;
}

IMM::~IMM() {}

void IMM::Initialize(const MeasurementPackage& meas_package) {
    // Initialize state based on first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        x_ << meas_package.raw_measurements_[0],
              meas_package.raw_measurements_[1],
              0, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double rho_dot = meas_package.raw_measurements_[2];
        
        x_ << rho * cos(phi),
              rho * sin(phi),
              rho_dot,
              0, 0;
    }
    
    // Initialize covariance
    P_ = MatrixXd::Identity(N_X, N_X);
    P_(0,0) = 1.0;
    P_(1,1) = 1.0;
    P_(2,2) = 10.0;
    P_(3,3) = 1.0;
    P_(4,4) = 1.0;
    
    // Initialize all four filters with same initial state
    cv_filter_.Initialize(x_, P_);
    ca_filter_.Initialize(x_, P_);
    ctrv_filter_.Initialize(x_, P_);
    ctra_filter_.Initialize(x_, P_);
    
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
}

void IMM::ProcessMeasurement(const MeasurementPackage& meas_package) {
    if (!is_initialized_) {
        Initialize(meas_package);
        return;
    }
    
    // Calculate time delta
    double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    
    // IMM Algorithm Steps:
    
    // Step 1: Calculate mixing probabilities and mix states
    MixStates();
    
    // Step 2: Filter-specific predictions
    Predict(dt);
    
    // Step 3: Filter-specific updates
    Update(meas_package);
    
    // Step 4: Update model probabilities
    std::array<double, N_MODELS> likelihoods;
    likelihoods[CV] = cv_filter_.CalculateLikelihood(meas_package);
    likelihoods[CA] = ca_filter_.CalculateLikelihood(meas_package);
    likelihoods[CTRV] = ctrv_filter_.CalculateLikelihood(meas_package);
    likelihoods[CTRA] = ctra_filter_.CalculateLikelihood(meas_package);
    UpdateModelProbabilities(likelihoods);
    
    // Step 5: Fuse estimates from all models
    FuseEstimate();
    
    std::cout << "IMM Probs: CV=" << mu_(CV) << " CA=" << mu_(CA) 
              << " CTRV=" << mu_(CTRV) << " CTRA=" << mu_(CTRA) << std::endl;
}

void IMM::MixStates() {
    // Calculate mixing probabilities mu_ij(i,j)
    // mu_ij(i,j) = P(model i at k-1 | model j at k)
    
    // Calculate normalization constants c_j
    for (int j = 0; j < N_MODELS; ++j) {
        c_j_(j) = 0.0;
        for (int i = 0; i < N_MODELS; ++i) {
            c_j_(j) += PI_(i, j) * mu_(i);
        }
    }
    
    // Calculate mixing probabilities
    for (int i = 0; i < N_MODELS; ++i) {
        for (int j = 0; j < N_MODELS; ++j) {
            if (c_j_(j) > 1e-10) {
                mu_ij_(i, j) = (PI_(i, j) * mu_(i)) / c_j_(j);
            } else {
                mu_ij_(i, j) = mu_(i);
            }
        }
    }
    
    // Get states from all filters (convert to common 5D representation)
    VectorXd x_cv = cv_filter_.GetState5D();
    VectorXd x_ca = ca_filter_.GetState5D();
    VectorXd x_ctrv = ctrv_filter_.x_;
    VectorXd x_ctra = ctra_filter_.GetState5D();
    
    // Get covariances in 5D format
    MatrixXd P_cv_5d = MatrixXd::Identity(5, 5);
    P_cv_5d.block(0, 0, 4, 4) = cv_filter_.P_;
    
    MatrixXd P_ca_5d = MatrixXd::Identity(5, 5);
    P_ca_5d.block(0, 0, 4, 4) = ca_filter_.P_.block(0, 0, 4, 4);
    
    MatrixXd P_ctrv = ctrv_filter_.P_;
    
    MatrixXd P_ctra = ctra_filter_.P_.block(0, 0, 5, 5);
    
    // Mix states for each model
    for (int j = 0; j < N_MODELS; ++j) {
        x_mixed_[j].fill(0.0);
        P_mixed_[j].fill(0.0);
        
        // Mix state estimates
        x_mixed_[j] += mu_ij_(CV, j) * x_cv;
        x_mixed_[j] += mu_ij_(CA, j) * x_ca;
        x_mixed_[j] += mu_ij_(CTRV, j) * x_ctrv;
        x_mixed_[j] += mu_ij_(CTRA, j) * x_ctra;
        
        // Mix covariances
        for (int i = 0; i < N_MODELS; ++i) {
            VectorXd x_diff;
            MatrixXd P_i;
            
            if (i == CV) {
                x_diff = x_cv - x_mixed_[j];
                P_i = P_cv_5d;
            } else if (i == CA) {
                x_diff = x_ca - x_mixed_[j];
                P_i = P_ca_5d;
            } else if (i == CTRV) {
                x_diff = x_ctrv - x_mixed_[j];
                P_i = P_ctrv;
            } else {  // CTRA
                x_diff = x_ctra - x_mixed_[j];
                P_i = P_ctra;
            }
            
            // Normalize angle difference
            NormalizeAngle(x_diff(3));
            
            P_mixed_[j] += mu_ij_(i, j) * (P_i + x_diff * x_diff.transpose());
        }
    }
    
    // Initialize each filter with its mixed state
    cv_filter_.SetStateFrom5D(x_mixed_[CV]);
    cv_filter_.P_ = P_mixed_[CV].block(0, 0, 4, 4);
    
    ca_filter_.SetStateFrom5D(x_mixed_[CA]);
    // CA has 6 state dimensions, extend the covariance
    MatrixXd P_ca_full = MatrixXd::Identity(6, 6);
    P_ca_full.block(0, 0, 5, 5) = P_mixed_[CA];
    ca_filter_.P_ = P_ca_full;
    
    ctrv_filter_.x_ = x_mixed_[CTRV];
    ctrv_filter_.P_ = P_mixed_[CTRV];
    
    ctra_filter_.SetStateFrom5D(x_mixed_[CTRA]);
    MatrixXd P_ctra_full = MatrixXd::Identity(6, 6);
    P_ctra_full.block(0, 0, 5, 5) = P_mixed_[CTRA];
    ctra_filter_.P_ = P_ctra_full;
}

void IMM::Predict(double dt) {
    // Each filter predicts with its own model
    cv_filter_.Predict(dt);
    ca_filter_.Predict(dt);
    ctrv_filter_.Predict(dt);
    ctra_filter_.Predict(dt);
}

void IMM::Update(const MeasurementPackage& meas_package) {
    // Each filter updates with the measurement
    cv_filter_.Update(meas_package);
    ca_filter_.Update(meas_package);
    ctrv_filter_.Update(meas_package);
    ctra_filter_.Update(meas_package);
}

void IMM::UpdateModelProbabilities(const std::array<double, N_MODELS>& likelihoods) {
    // Update model probabilities using Bayes' theorem
    // mu_j(k) = c_j * L_j
    
    VectorXd new_mu(N_MODELS);
    for (int j = 0; j < N_MODELS; ++j) {
        new_mu(j) = c_j_(j) * likelihoods[j];
    }
    
    // Normalize
    double sum = new_mu.sum();
    if (sum > 1e-10) {
        mu_ = new_mu / sum;
    }
    
    // Prevent probabilities from becoming too small
    for (int i = 0; i < N_MODELS; ++i) {
        if (mu_(i) < 0.001) mu_(i) = 0.001;
    }
    
    // Re-normalize
    mu_ = mu_ / mu_.sum();
}

void IMM::FuseEstimate() {
    // Combine estimates from all models weighted by their probabilities
    x_.fill(0.0);
    P_.fill(0.0);
    
    // Get states from filters
    VectorXd x_cv = cv_filter_.GetState5D();
    VectorXd x_ca = ca_filter_.GetState5D();
    VectorXd x_ctrv = ctrv_filter_.x_;
    VectorXd x_ctra = ctra_filter_.GetState5D();
    
    // Fuse states
    x_ = mu_(CV) * x_cv + mu_(CA) * x_ca + mu_(CTRV) * x_ctrv + mu_(CTRA) * x_ctra;
    
    // Fuse covariances
    MatrixXd P_cv_5d = MatrixXd::Identity(5, 5);
    P_cv_5d.block(0, 0, 4, 4) = cv_filter_.P_;
    
    MatrixXd P_ca_5d = MatrixXd::Identity(5, 5);
    P_ca_5d.block(0, 0, 4, 4) = ca_filter_.P_.block(0, 0, 4, 4);
    
    MatrixXd P_ctra = ctra_filter_.P_.block(0, 0, 5, 5);
    
    for (int i = 0; i < N_MODELS; ++i) {
        VectorXd x_diff;
        MatrixXd P_i;
        
        if (i == CV) {
            x_diff = x_cv - x_;
            P_i = P_cv_5d;
        } else if (i == CA) {
            x_diff = x_ca - x_;
            P_i = P_ca_5d;
        } else if (i == CTRV) {
            x_diff = x_ctrv - x_;
            P_i = ctrv_filter_.P_;
        } else {  // CTRA
            x_diff = x_ctra - x_;
            P_i = P_ctra;
        }
        
        // Normalize angle difference
        NormalizeAngle(x_diff(3));
        
        P_ += mu_(i) * (P_i + x_diff * x_diff.transpose());
    }
}

void IMM::NormalizeAngle(double& angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}
