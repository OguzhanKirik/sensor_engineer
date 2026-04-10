#include "cv_ekf.h"
#include <cmath>
#include <iostream>

CV_EKF::CV_EKF() {
    // State dimension is 4: [px, py, vx, vy]
    x_ = VectorXd(4);
    x_.fill(0.0);
    
    // Covariance matrix
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    
    // Lidar measurement matrix (measures px, py)
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    // Measurement noise
    std_laspx_ = 0.15;
    std_laspy_ = 0.15;
    std_radr_ = 0.3;
    std_radphi_ = 0.03;
    std_radrd_ = 0.3;
    
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;
    
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;
    
    // Process noise
    std_a_ = 1.0;  // Lower for CV model
    
    is_initialized_ = false;
}

CV_EKF::~CV_EKF() {}

void CV_EKF::Initialize(const VectorXd& x_in, const MatrixXd& P_in) {
    if (x_in.size() == 4) {
        x_ = x_in;
    } else if (x_in.size() == 5) {
        SetStateFrom5D(x_in);
    }
    P_ = P_in.block(0, 0, 4, 4);
    is_initialized_ = true;
}

void CV_EKF::Predict(double dt) {
    // State transition matrix for CV model
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    
    // Process noise covariance
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    MatrixXd Q = MatrixXd(4, 4);
    Q << dt4/4, 0, dt3/2, 0,
         0, dt4/4, 0, dt3/2,
         dt3/2, 0, dt2, 0,
         0, dt3/2, 0, dt2;
    Q *= std_a_ * std_a_;
    
    // Predict
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void CV_EKF::Update(const MeasurementPackage& meas_package) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }
}

void CV_EKF::UpdateLidar(const MeasurementPackage& meas_package) {
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = z - z_pred;
    
    MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
    MatrixXd K = P_ * H_laser_.transpose() * S.inverse();
    
    // Update state and covariance
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_laser_) * P_;
}

void CV_EKF::UpdateRadar(const MeasurementPackage& meas_package) {
    // Extract state
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    
    // Predicted measurement
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double rho_dot = (px*vx + py*vy) / std::max(rho, 0.0001);
    
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    
    // Measurement residual
    VectorXd y = meas_package.raw_measurements_ - z_pred;
    
    // Normalize angle
    while (y(1) > M_PI) y(1) -= 2.0 * M_PI;
    while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
    
    // Jacobian matrix
    MatrixXd Hj(3, 4);
    double rho2 = rho * rho;
    double rho3 = rho2 * rho;
    
    if (fabs(rho) < 0.0001) {
        rho = 0.0001;
        rho2 = rho * rho;
        rho3 = rho2 * rho;
    }
    
    Hj << px/rho, py/rho, 0, 0,
          -py/rho2, px/rho2, 0, 0,
          py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;
    
    MatrixXd S = Hj * P_ * Hj.transpose() + R_radar_;
    MatrixXd K = P_ * Hj.transpose() * S.inverse();
    
    // Update
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * Hj) * P_;
}

double CV_EKF::CalculateLikelihood(const MeasurementPackage& meas_package) {
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_pred;
    MatrixXd H;
    MatrixXd R;
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        z_pred = H_laser_ * x_;
        H = H_laser_;
        R = R_laser_;
    } else {
        double px = x_(0);
        double py = x_(1);
        double vx = x_(2);
        double vy = x_(3);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*vx + py*vy) / std::max(rho, 0.0001);
        
        z_pred = VectorXd(3);
        z_pred << rho, phi, rho_dot;
        
        // Use Jacobian
        double rho2 = rho * rho;
        double rho3 = rho2 * rho;
        
        H = MatrixXd(3, 4);
        H << px/rho, py/rho, 0, 0,
             -py/rho2, px/rho2, 0, 0,
             py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;
        
        R = R_radar_;
    }
    
    VectorXd y = z - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        while (y(1) > M_PI) y(1) -= 2.0 * M_PI;
        while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
    }
    
    MatrixXd S = H * P_ * H.transpose() + R;
    
    // Likelihood: exp(-0.5 * y^T * S^-1 * y) / sqrt(det(2*pi*S))
    double det_S = S.determinant();
    if (det_S < 1e-10) det_S = 1e-10;
    
    double exponent = -0.5 * y.transpose() * S.inverse() * y;
    double likelihood = exp(exponent) / sqrt(pow(2*M_PI, z.size()) * det_S);
    
    return likelihood;
}

VectorXd CV_EKF::GetState5D() const {
    VectorXd x_5d(5);
    double v = sqrt(x_(2)*x_(2) + x_(3)*x_(3));
    double yaw = atan2(x_(3), x_(2));
    
    x_5d << x_(0), x_(1), v, yaw, 0.0;  // yaw_rate is 0 for CV
    return x_5d;
}

void CV_EKF::SetStateFrom5D(const VectorXd& x_5d) {
    double v = x_5d(2);
    double yaw = x_5d(3);
    
    x_(0) = x_5d(0);  // px
    x_(1) = x_5d(1);  // py
    x_(2) = v * cos(yaw);  // vx
    x_(3) = v * sin(yaw);  // vy
}
