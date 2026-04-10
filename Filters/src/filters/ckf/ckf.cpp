#include "ckf.h"
#include <iostream>
#include <cmath>

CKF::CKF() {
    // State dimension
    n_x_ = 5;

    // Number of cubature points: 2 * n_x
    n_sigma_ = 2 * n_x_;

    // Initialize state vector
    x_ = VectorXd(n_x_);
    x_.fill(0.0);

    // Initialize covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_.fill(0.0);

    // Initialize weights (all equal for CKF)
    weights_ = VectorXd(n_sigma_);
    double weight = 1.0 / (double)n_sigma_;
    for (int i = 0; i < n_sigma_; i++) {
        weights_(i) = weight;
    }

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.8;  // Reduced for better tracking

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.6;  // Slightly increased for turn sensitivity

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    // Initialize predicted cubature points
    Xsig_pred_ = MatrixXd(n_x_, n_sigma_);

    // Not initialized yet
    is_initialized_ = false;

    time_us_ = 0;
}

CKF::~CKF() {}

void CKF::Initialize(const VectorXd& x, const MatrixXd& P) {
    x_ = x;
    P_ = P;
    is_initialized_ = true;
}

void CKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
    if (!is_initialized_) {
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
        P_ = MatrixXd::Identity(n_x_, n_x_);
        P_(0,0) = 0.5;   // Reduced from 1.0 for better tracking
        P_(1,1) = 0.5;   // Reduced from 1.0 for better tracking
        P_(2,2) = 1.0;   // Reduced from 10.0 for faster convergence
        P_(3,3) = 1.0;
        P_(4,4) = 1.0;

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    // Calculate delta_t
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    // Prediction step
    Predict(delta_t);

    // Update step
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }
}

MatrixXd CKF::GenerateCubaturePoints(const VectorXd& x, const MatrixXd& P) {
    // CKF uses spherical-radial cubature rule
    // Number of cubature points: 2n
    // Points are generated as: x ± √n * [√P]_i
    
    MatrixXd Xsig = MatrixXd(n_x_, n_sigma_);
    
    // Calculate square root of P using Cholesky decomposition
    MatrixXd A = P.llt().matrixL();
    
    // Scale factor: sqrt(n_x)
    double scale = sqrt((double)n_x_);
    
    // Generate cubature points
    for (int i = 0; i < n_x_; i++) {
        Xsig.col(i)       = x + scale * A.col(i);
        Xsig.col(i + n_x_) = x - scale * A.col(i);
    }
    
    return Xsig;
}

VectorXd CKF::ProcessModel(const VectorXd& x, double delta_t, double nu_a, double nu_yawdd) {
    // CTRV model with process noise
    VectorXd x_pred = VectorXd(n_x_);
    
    double px = x(0);
    double py = x(1);
    double v = x(2);
    double yaw = x(3);
    double yawd = x(4);
    
    // Predicted state values
    double px_p, py_p;
    
    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = py + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
    } else {
        px_p = px + v*delta_t*cos(yaw);
        py_p = py + v*delta_t*sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    // Add noise
    px_p += 0.5*nu_a*delta_t*delta_t*cos(yaw);
    py_p += 0.5*nu_a*delta_t*delta_t*sin(yaw);
    v_p += nu_a*delta_t;
    
    yaw_p += 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p += nu_yawdd*delta_t;
    
    x_pred << px_p, py_p, v_p, yaw_p, yawd_p;
    
    return x_pred;
}

void CKF::PredictCubaturePoints(const MatrixXd& Xsig, double delta_t) {
    // Predict each cubature point through the process model
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd x = Xsig.col(i);
        
        // Add process noise (zero mean for prediction)
        // Note: For CKF, we handle noise in the covariance directly
        Xsig_pred_.col(i) = ProcessModel(x, delta_t, 0, 0);
    }
}

void CKF::Predict(double delta_t) {
    // Step 1: Generate cubature points
    MatrixXd Xsig = GenerateCubaturePoints(x_, P_);
    
    // Step 2: Propagate cubature points through process model
    PredictCubaturePoints(Xsig, delta_t);
    
    // Step 3: Predict mean
    x_.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
    
    // Step 4: Predict covariance
    P_.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // Normalize angle
        NormalizeAngle(x_diff(3));
        
        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
    
    // Step 5: Add process noise covariance
    // For CTRV, we add process noise directly to state covariance
    // This accounts for acceleration and yaw acceleration uncertainty
    MatrixXd Q = MatrixXd(n_x_, n_x_);
    Q.fill(0.0);
    
    // Position noise (from acceleration uncertainty)
    double dt2 = delta_t * delta_t;
    double dt3 = dt2 * delta_t;
    double dt4 = dt3 * delta_t;
    
    // Simplified process noise for CTRV
    Q(0,0) = dt4/4 * std_a_*std_a_;  // px noise from acceleration
    Q(1,1) = dt4/4 * std_a_*std_a_;  // py noise from acceleration
    Q(2,2) = dt2 * std_a_*std_a_;     // v noise from acceleration
    Q(3,3) = dt4/4 * std_yawdd_*std_yawdd_;  // yaw noise from yaw acceleration
    Q(4,4) = dt2 * std_yawdd_*std_yawdd_;    // yaw_rate noise from yaw acceleration
    
    P_ += Q;
}

void CKF::UpdateLidar(const MeasurementPackage& meas_package) {
    // Lidar measurement dimension (px, py)
    int n_z = 2;
    
    // Generate cubature points from predicted state
    MatrixXd Xsig = GenerateCubaturePoints(x_, P_);
    
    // Transform cubature points into measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sigma_);
    for (int i = 0; i < n_sigma_; i++) {
        double px = Xsig(0, i);
        double py = Xsig(1, i);
        
        Zsig(0, i) = px;
        Zsig(1, i) = py;
    }
    
    // Calculate mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    // Calculate measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    // Add measurement noise
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_, 0,
         0, std_laspy_*std_laspy_;
    S += R;
    
    // Calculate cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd x_diff = Xsig.col(i) - x_;
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        NormalizeAngle(x_diff(3));
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // Calculate Kalman gain
    MatrixXd K = Tc * S.inverse();
    
    // Update state and covariance
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;
    
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
}

void CKF::UpdateRadar(const MeasurementPackage& meas_package) {
    // Radar measurement dimension (rho, phi, rho_dot)
    int n_z = 3;
    
    // Generate cubature points from predicted state
    MatrixXd Xsig = GenerateCubaturePoints(x_, P_);
    
    // Transform cubature points into measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sigma_);
    for (int i = 0; i < n_sigma_; i++) {
        double px = Xsig(0, i);
        double py = Xsig(1, i);
        double v = Xsig(2, i);
        double yaw = Xsig(3, i);
        
        double vx = v * cos(yaw);
        double vy = v * sin(yaw);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*vx + py*vy) / (rho + 1e-6);
        
        Zsig(0, i) = rho;
        Zsig(1, i) = phi;
        Zsig(2, i) = rho_dot;
    }
    
    // Calculate mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    // Calculate measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        NormalizeAngle(z_diff(1));  // Normalize phi
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    // Add measurement noise
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_*std_radr_, 0, 0,
         0, std_radphi_*std_radphi_, 0,
         0, 0, std_radrd_*std_radrd_;
    S += R;
    
    // Calculate cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd x_diff = Xsig.col(i) - x_;
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        NormalizeAngle(x_diff(3));
        NormalizeAngle(z_diff(1));
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // Calculate Kalman gain
    MatrixXd K = Tc * S.inverse();
    
    // Update state and covariance
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;
    NormalizeAngle(z_diff(1));
    
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
}

double CKF::CalculateLikelihood(const MeasurementPackage& meas_package) {
    int n_z = (meas_package.sensor_type_ == MeasurementPackage::LASER) ? 2 : 3;
    
    // Generate cubature points
    MatrixXd Xsig = GenerateCubaturePoints(x_, P_);
    
    // Transform to measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sigma_);
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        for (int i = 0; i < n_sigma_; i++) {
            Zsig(0, i) = Xsig(0, i);
            Zsig(1, i) = Xsig(1, i);
        }
    } else {  // RADAR
        for (int i = 0; i < n_sigma_; i++) {
            double px = Xsig(0, i);
            double py = Xsig(1, i);
            double v = Xsig(2, i);
            double yaw = Xsig(3, i);
            
            double vx = v * cos(yaw);
            double vy = v * sin(yaw);
            double rho = sqrt(px*px + py*py);
            
            Zsig(0, i) = rho;
            Zsig(1, i) = atan2(py, px);
            Zsig(2, i) = (px*vx + py*vy) / (rho + 1e-6);
        }
    }
    
    // Predicted measurement mean
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    // Measurement covariance
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        if (n_z == 3) NormalizeAngle(z_diff(1));
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    // Add measurement noise
    if (n_z == 2) {
        MatrixXd R = MatrixXd(n_z, n_z);
        R << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;
        S += R;
    } else {
        MatrixXd R = MatrixXd(n_z, n_z);
        R << std_radr_*std_radr_, 0, 0,
             0, std_radphi_*std_radphi_, 0,
             0, 0, std_radrd_*std_radrd_;
        S += R;
    }
    
    // Calculate likelihood
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;
    if (n_z == 3) NormalizeAngle(z_diff(1));
    
    double det_S = S.determinant();
    double exponent = -0.5 * z_diff.transpose() * S.inverse() * z_diff;
    double likelihood = (1.0 / sqrt(pow(2*M_PI, n_z) * det_S)) * exp(exponent);
    
    return likelihood;
}

void CKF::NormalizeAngle(double& angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}
