#include "ctrv_ukf.h"
#include <iostream>

CTRV_UKF::CTRV_UKF() {
    x_ = Eigen::VectorXd(5);
    x_.fill(0.0);
    
    P_ = Eigen::MatrixXd(5, 5);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    is_initialized_ = false;
}

CTRV_UKF::~CTRV_UKF() {}

void CTRV_UKF::Initialize(const Eigen::VectorXd& x_in, const Eigen::MatrixXd& P_in) {
    ukf_.x_ = x_in;
    ukf_.P_ = P_in;
    ukf_.is_initialized_ = true;
    is_initialized_ = true;
    
    x_ = x_in;
    P_ = P_in;
}

void CTRV_UKF::Predict(double dt) {
    ukf_.Prediction(dt);
    x_ = ukf_.x_;
    P_ = ukf_.P_;
}

void CTRV_UKF::Update(const MeasurementPackage& meas_package) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        ukf_.UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        ukf_.UpdateRadar(meas_package);
    }
    
    x_ = ukf_.x_;
    P_ = ukf_.P_;
}

double CTRV_UKF::CalculateLikelihood(const MeasurementPackage& meas_package) {
    using Eigen::VectorXd;
    using Eigen::MatrixXd;
    
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_pred;
    MatrixXd S;
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        // Lidar measurement
        z_pred = VectorXd(2);
        z_pred << ukf_.x_(0), ukf_.x_(1);
        
        S = MatrixXd(2, 2);
        S << ukf_.std_laspx_ * ukf_.std_laspx_, 0,
             0, ukf_.std_laspy_ * ukf_.std_laspy_;
        S += ukf_.P_.block(0, 0, 2, 2);
        
    } else {
        // Radar measurement
        double px = ukf_.x_(0);
        double py = ukf_.x_(1);
        double v = ukf_.x_(2);
        double yaw = ukf_.x_(3);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / std::max(rho, 0.0001);
        
        z_pred = VectorXd(3);
        z_pred << rho, phi, rho_dot;
        
        // Simplified S calculation (should use measurement prediction from sigma points)
        S = MatrixXd(3, 3);
        S << ukf_.std_radr_ * ukf_.std_radr_, 0, 0,
             0, ukf_.std_radphi_ * ukf_.std_radphi_, 0,
             0, 0, ukf_.std_radrd_ * ukf_.std_radrd_;
    }
    
    VectorXd y = z - z_pred;
    
    // Normalize angle for radar
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        while (y(1) > M_PI) y(1) -= 2.0 * M_PI;
        while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
    }
    
    // Calculate likelihood
    double det_S = S.determinant();
    if (det_S < 1e-10) det_S = 1e-10;
    
    double exponent = -0.5 * y.transpose() * S.inverse() * y;
    double likelihood = exp(exponent) / sqrt(pow(2*M_PI, z.size()) * det_S);
    
    return likelihood;
}
