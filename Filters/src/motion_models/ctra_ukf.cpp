#include "ctra_ukf.h"
#include <cmath>
#include <iostream>

CTRA_UKF::CTRA_UKF() {
    // State dimension: [px, py, v, yaw, yaw_rate, a]
    n_x_ = 6;
    
    // Augmented dimension (state + 2 process noises)
    n_aug_ = 8;
    
    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;
    
    // Initialize state
    x_ = VectorXd(n_x_);
    x_.fill(0.0);
    
    // Initialize covariance
    P_ = MatrixXd::Identity(n_x_, n_x_);
    
    // Sigma point weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
    
    // Predicted sigma points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Measurement noise
    std_laspx_ = 0.15;
    std_laspy_ = 0.15;
    std_radr_ = 0.3;
    std_radphi_ = 0.03;
    std_radrd_ = 0.3;
    
    // Process noise
    std_a_ = 1.5;      // Acceleration noise
    std_yawdd_ = 0.6;  // Yaw acceleration noise
    
    is_initialized_ = false;
}

CTRA_UKF::~CTRA_UKF() {}

void CTRA_UKF::Initialize(const VectorXd& x_in, const MatrixXd& P_in) {
    if (x_in.size() == 6) {
        x_ = x_in;
    } else if (x_in.size() == 5) {
        SetStateFrom5D(x_in);
    }
    
    if (P_in.rows() == 6) {
        P_ = P_in;
    } else if (P_in.rows() == 5) {
        P_ = MatrixXd::Identity(6, 6);
        P_.block(0, 0, 5, 5) = P_in;
    } else {
        P_ = MatrixXd::Identity(6, 6);
    }
    
    is_initialized_ = true;
}

void CTRA_UKF::Predict(double dt) {
    // Generate augmented sigma points
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;      // mean of acceleration noise
    x_aug(n_x_ + 1) = 0;  // mean of yaw accel noise
    
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;
    
    // Calculate square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    
    // Create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.col(0) = x_aug;
    
    double sqrt_lambda_n = sqrt(lambda_ + n_aug_);
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug.col(i + 1) = x_aug + sqrt_lambda_n * A.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n * A.col(i);
    }
    
    // Predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        double px = Xsig_aug(0, i);
        double py = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double a = Xsig_aug(5, i);
        double nu_a = Xsig_aug(6, i);
        double nu_yawdd = Xsig_aug(7, i);
        
        // Predicted state values
        double px_p, py_p, v_p, yaw_p, yawd_p, a_p;
        
        // CTRA motion model
        if (fabs(yawd) > 0.001) {
            px_p = px + (v * sin(yaw + yawd * dt) - v * sin(yaw)) / yawd 
                   + 0.5 * dt * dt * cos(yaw) * a;
            py_p = py + (v * (-cos(yaw + yawd * dt) + cos(yaw))) / yawd 
                   + 0.5 * dt * dt * sin(yaw) * a;
        } else {
            px_p = px + v * cos(yaw) * dt + 0.5 * dt * dt * cos(yaw) * a;
            py_p = py + v * sin(yaw) * dt + 0.5 * dt * dt * sin(yaw) * a;
        }
        
        v_p = v + a * dt;
        yaw_p = yaw + yawd * dt;
        yawd_p = yawd;
        a_p = a;
        
        // Add noise
        px_p += 0.5 * dt * dt * cos(yaw) * nu_a;
        py_p += 0.5 * dt * dt * sin(yaw) * nu_a;
        v_p += dt * nu_a;
        yaw_p += 0.5 * dt * dt * nu_yawdd;
        yawd_p += dt * nu_yawdd;
        
        // Write predicted sigma points
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
        Xsig_pred_(5, i) = a_p;
    }
    
    // Predict mean and covariance
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
    
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}

void CTRA_UKF::Update(const MeasurementPackage& meas_package) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }
}

void CTRA_UKF::UpdateLidar(const MeasurementPackage& meas_package) {
    int n_z = 2;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
    }
    
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_ * std_laspx_, 0,
         0, std_laspy_ * std_laspy_;
    S += R;
    
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    MatrixXd K = Tc * S.inverse();
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
    
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
}

void CTRA_UKF::UpdateRadar(const MeasurementPackage& meas_package) {
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / std::max(rho, 0.001);
        
        Zsig(0, i) = rho;
        Zsig(1, i) = phi;
        Zsig(2, i) = rho_dot;
    }
    
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;
        
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_ * std_radr_, 0, 0,
         0, std_radphi_ * std_radphi_, 0,
         0, 0, std_radrd_ * std_radrd_;
    S += R;
    
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;
        
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    MatrixXd K = Tc * S.inverse();
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;
    
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
}

double CTRA_UKF::CalculateLikelihood(const MeasurementPackage& meas_package) {
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_pred;
    MatrixXd S;
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        z_pred = VectorXd(2);
        z_pred << x_(0), x_(1);
        
        S = MatrixXd(2, 2);
        S << std_laspx_ * std_laspx_, 0,
             0, std_laspy_ * std_laspy_;
        S += P_.block(0, 0, 2, 2);
    } else {
        double px = x_(0);
        double py = x_(1);
        double v = x_(2);
        double yaw = x_(3);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / std::max(rho, 0.001);
        
        z_pred = VectorXd(3);
        z_pred << rho, phi, rho_dot;
        
        S = MatrixXd(3, 3);
        S << std_radr_ * std_radr_, 0, 0,
             0, std_radphi_ * std_radphi_, 0,
             0, 0, std_radrd_ * std_radrd_;
    }
    
    VectorXd y = z - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        while (y(1) > M_PI) y(1) -= 2.0 * M_PI;
        while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
    }
    
    double det_S = S.determinant();
    if (det_S < 1e-10) det_S = 1e-10;
    
    double exponent = -0.5 * y.transpose() * S.inverse() * y;
    double likelihood = exp(exponent) / sqrt(pow(2*M_PI, z.size()) * det_S);
    
    return likelihood;
}

VectorXd CTRA_UKF::GetState5D() const {
    VectorXd x_5d(5);
    x_5d << x_(0), x_(1), x_(2), x_(3), x_(4);
    return x_5d;
}

void CTRA_UKF::SetStateFrom5D(const VectorXd& x_5d) {
    x_(0) = x_5d(0);  // px
    x_(1) = x_5d(1);  // py
    x_(2) = x_5d(2);  // v
    x_(3) = x_5d(3);  // yaw
    x_(4) = x_5d(4);  // yaw_rate
    x_(5) = 0.0;      // acceleration starts at 0
}
