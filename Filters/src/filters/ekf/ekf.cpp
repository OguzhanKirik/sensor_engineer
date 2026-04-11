#include"ekf.hpp"

// EKF forward pass for the 5D CTRV highway tracking model.

EKF::EKF(){
    // ==========================================
    // Initialize Process Noise Parameters
    // ==========================================
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.6;

    // ==========================================
    // Initialize Measurement Noise Parameters
    // ==========================================
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
    
    // ==========================================
    // Initialize Filter State Variables
    // ==========================================
    _is_initilized =false;
    use_laser_ = true;
    use_radar_ = true;
    time_us_ = 0;
    
    // State vector [px, py, v, yaw, yaw_rate]
    x_ = VectorXd(5);
    // State covariance matrix
    P_ = MatrixXd(5,5);
    // State transition matrix
    F_ = MatrixXd(5,5);
    
    // ==========================================
    // Initialize Radar Measurement Matrices
    // ==========================================
    R_radar_ = MatrixXd(3, 3);  // Radar measurement noise covariance
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;
    H_radar_ = MatrixXd(3, 5);  // Radar measurement Jacobian (computed dynamically)

    // ==========================================
    // Initialize Lidar Measurement Matrices
    // ==========================================
    // Lidar measurement matrix: extracts px and py from state vector
    H_laser_ = MatrixXd(2, 5);
    H_laser_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0;
    // Lidar measurement noise covariance
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    // Process noise covariance matrix (computed dynamically in Prediction)
    Q_ = MatrixXd(5,5);
    last_x_pred_ = VectorXd(5);
    last_P_pred_ = MatrixXd(5, 5);
    last_F_jacobian_ = MatrixXd::Identity(5, 5);
    
};

EKF::~EKF() {}

void EKF::ClearStepHistory() {
    // Smoothers replay a fresh log and then consume this cache offline.
    step_history_.clear();
}

const std::vector<EKFRTSStepData>& EKF::GetStepHistory() const {
    return step_history_;
}

void EKF::NormalizeAngle(double& angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}


Eigen::MatrixXd EKF::CalculateRadarJacobian(const Eigen::VectorXd& x_state) {
    // Initialize Jacobian matrix (3 measurements: rho, phi, rho_dot) x (5 state variables)
    Eigen::MatrixXd Hj(3, 5);
    Hj.setZero();

    // Extract state variables
    const double px   = x_state(0);
    const double py   = x_state(1);
    const double v    = x_state(2);
    const double yaw  = x_state(3);

    // Check for division by zero - if too close to origin, return zero Jacobian
    const double eps = 1e-6;
    const double c1 = px * px + py * py;  // Squared distance from origin

    if (c1 < eps) {
        return Hj;  // Return zero matrix if position is near origin
    }

    // Precompute commonly used terms
    const double rho = sqrt(c1);          // Range (distance from origin)
    const double c3 = c1 * rho;           // c1^(3/2) used in range rate derivatives

    const double cos_yaw = cos(yaw);
    const double sin_yaw = sin(yaw);

    // Velocity projection onto position vector
    const double proj = px * v * cos_yaw + py * v * sin_yaw;

    // ==========================================
    // Fill Jacobian Matrix: dh/dx
    // ==========================================
    
    // Row 0: Derivatives of rho (range) with respect to state variables
    Hj(0, 0) = px / rho;  // d(rho)/d(px)
    Hj(0, 1) = py / rho;  // d(rho)/d(py)

    // Row 1: Derivatives of phi (bearing angle) with respect to state variables
    Hj(1, 0) = -py / c1;  // d(phi)/d(px)
    Hj(1, 1) =  px / c1;  // d(phi)/d(py)

    // Row 2: Derivatives of rho_dot (range rate) with respect to state variables
    Hj(2, 0) = (v * cos_yaw) / rho - (proj * px) / c3;      // d(rho_dot)/d(px)
    Hj(2, 1) = (v * sin_yaw) / rho - (proj * py) / c3;      // d(rho_dot)/d(py)
    Hj(2, 2) = (px * cos_yaw + py * sin_yaw) / rho;         // d(rho_dot)/d(v)
    Hj(2, 3) = v * (-px * sin_yaw + py * cos_yaw) / rho;    // d(rho_dot)/d(yaw)
    // Hj(2, 4) = 0  (range rate doesn't depend on yaw rate)

    return Hj;
}

void EKF::ProcessMeasurument(MeasurementPackage meas_package){
    // ==========================================
    // INITIALIZATION: First Measurement
    // ==========================================
    if(_is_initilized == false){
        _is_initilized = true;
        time_us_ = meas_package.timestamp_;
        
        if(meas_package.sensor_type_ == MeasurementPackage::LASER){
            // Initialize state from lidar: we get px, py directly
            // Set v, yaw, yaw_rate to zero (unknown from first measurement)
            x_ << meas_package.raw_measurements_(0),
             meas_package.raw_measurements_(1),
             0,0,0;
            
            // Initialize covariance: high certainty for px, py; low certainty for others
            P_ << std_laspx_*std_laspx_,0,0,0,0,
                    0,std_laspy_*std_laspy_,0,0,0,
                    0,0,1,0,0,
                    0,0,0,1,0,
                    0,0,0,0,1;
        }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
            // Extract radar measurements in polar coordinates
            double rho = meas_package.raw_measurements_(0);      // range
            double phi = meas_package.raw_measurements_(1);      // bearing
            double rho_dot = meas_package.raw_measurements_(2);  // range rate

            // Convert polar to Cartesian coordinates for state initialization
            (void)rho_dot;
            x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;

            // Initialize covariance: uncertainty based on radar noise
            P_ << std_radr_*std_radr_,0,0,0,0,
                    0,std_radr_*std_radr_,0,0,0,
                    0,0,1,0,0,
                    0,0,0,1,0,
                    0,0,0,0,1;
        }

        EKFRTSStepData init_step;
        init_step.timestamp = time_us_;
        init_step.is_initialization = true;
        init_step.x_filtered = x_;
        init_step.P_filtered = P_;
        init_step.x_predicted = x_;
        init_step.P_predicted = P_;
        init_step.F_jacobian = MatrixXd::Identity(5, 5);
        step_history_.push_back(init_step);
        return; // Exit after initialization - don't predict or update yet
    }

    // ==========================================
    // PREDICTION STEP: Propagate state forward in time
    // ==========================================
    // Compute time elapsed since last measurement (convert microseconds to seconds)
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    // Predict state and covariance using CTRV motion model
    Prediction(dt);

    // ==========================================
    // UPDATE STEP: Correct prediction using measurement
    // ==========================================
    if (meas_package.sensor_type_ == MeasurementPackage::LASER ){
        UpdateLidar(meas_package);  // Linear update using lidar measurement
    }else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
        UpdateRadar(meas_package);  // Nonlinear update using radar measurement
    }

    // Persist the posterior and local linearization terms needed by the
    // backward EKF smoothers.
    EKFRTSStepData step;
    step.timestamp = meas_package.timestamp_;
    step.is_initialization = false;
    step.x_filtered = x_;
    step.P_filtered = P_;
    step.x_predicted = last_x_pred_;
    step.P_predicted = last_P_pred_;
    step.F_jacobian = last_F_jacobian_;
    step_history_.push_back(step);

}

void EKF::Prediction(double dt) {
  const double px   = x_(0);
  const double py   = x_(1);
  const double v    = x_(2);
  const double yaw  = x_(3);
  const double yawd = x_(4);

  const double eps = 1e-6;

  // 1) Predict state with nonlinear CTRV equations
  VectorXd x_pred(5);

  if (fabs(yawd) > eps) {
    const double yaw_new = yaw + yawd * dt;

    x_pred(0) = px + (v / yawd) * (sin(yaw_new) - sin(yaw));
    x_pred(1) = py + (v / yawd) * (-cos(yaw_new) + cos(yaw));
    x_pred(2) = v;
    x_pred(3) = yaw_new;
    x_pred(4) = yawd;
  } else {
    x_pred(0) = px + v * cos(yaw) * dt;
    x_pred(1) = py + v * sin(yaw) * dt;
    x_pred(2) = v;
    x_pred(3) = yaw + yawd * dt;
    x_pred(4) = yawd;
  }

  x_ = x_pred;
  NormalizeAngle(x_(3));
  last_x_pred_ = x_;

  // 2) Build the local process linearization used by both the EKF covariance
  // update and the backward RTS/fixed-lag smoothers.
  MatrixXd Fj = MatrixXd::Identity(5, 5);

  if (fabs(yawd) > eps) {
    const double yaw_new = yaw + yawd * dt;
    const double s1 = sin(yaw);
    const double c1 = cos(yaw);
    const double s2 = sin(yaw_new);
    const double c2 = cos(yaw_new);

    Fj(0, 2) = (s2 - s1) / yawd;
    Fj(0, 3) = (v / yawd) * (c2 - c1);
    Fj(0, 4) = v * ((dt * c2) / yawd - (s2 - s1) / (yawd * yawd));

    Fj(1, 2) = (-c2 + c1) / yawd;
    Fj(1, 3) = (v / yawd) * (s2 - s1);
    Fj(1, 4) = v * ((dt * s2) / yawd - (-c2 + c1) / (yawd * yawd));

    Fj(3, 4) = dt;
  } else {
    Fj(0, 2) = cos(yaw) * dt;
    Fj(0, 3) = -v * sin(yaw) * dt;

    Fj(1, 2) = sin(yaw) * dt;
    Fj(1, 3) =  v * cos(yaw) * dt;

    Fj(3, 4) = dt;
  }

  // 3) Map longitudinal and yaw acceleration noise into state space.
  MatrixXd G = MatrixXd::Zero(5, 2);

  G(0, 0) = 0.5 * dt * dt * cos(yaw);
  G(1, 0) = 0.5 * dt * dt * sin(yaw);
  G(2, 0) = dt;

  G(3, 1) = 0.5 * dt * dt;
  G(4, 1) = dt;

  MatrixXd Qv(2, 2);
  Qv << std_a_ * std_a_, 0,
        0, std_yawdd_ * std_yawdd_;

  Q_ = G * Qv * G.transpose();

  // 4) Standard EKF covariance propagation.
  P_ = Fj * P_ * Fj.transpose() + Q_;
  last_P_pred_ = P_;
  last_F_jacobian_ = Fj;
}

void EKF::UpdateLidar(const MeasurementPackage& meas_package){
    
    // Extract Lidar Measurement
    Eigen::VectorXd z(2);  // Lidar measures [px, py]
    z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

    // Compute Innovation (measurement residual)
    Eigen::VectorXd z_pred = H_laser_ * x_;  // Predicted measurement from current state
    Eigen::VectorXd y = z - z_pred;           // Innovation: difference between actual and predicted

    // Calculate Kalman Gain
    Eigen::MatrixXd Ht = H_laser_.transpose();
    Eigen::MatrixXd S = H_laser_ * P_ * Ht + R_laser_;  // Innovation covariance
    Eigen::MatrixXd K = P_ * Ht * S.inverse();          // Kalman gain

    // Update State and Covariance
    x_ = x_ + K * y;  // Correct state estimate using innovation

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    P_ = (I - K * H_laser_) * P_;  // Update covariance (Joseph form)
}

void EKF::UpdateRadar(const MeasurementPackage& meas_package) {
    // Extract Radar Measurement
    Eigen::VectorXd z = meas_package.raw_measurements_;  // [rho, phi, rho_dot]

    // Predict Radar Measurement from Current State
    const double px = x_(0);
    const double py = x_(1);
    const double v = x_(2);
    const double yaw = x_(3);

    // Convert velocity to Cartesian components
    const double vx = v * cos(yaw);
    const double vy = v * sin(yaw);

    // Compute predicted radar measurements in polar coordinates
    const double eps = 1e-6;
    const double rho2 = px * px + py * py;
    const double rho = sqrt(std::max(rho2, eps));  // Prevent division by zero

    Eigen::VectorXd z_pred(3);
    z_pred << rho,                          // predicted range
              atan2(py, px),                // predicted bearing
              (px * vx + py * vy) / rho;    // predicted range rate

    // Compute Innovation (measurement residual)
    Eigen::VectorXd y = z - z_pred;
    NormalizeAngle(y(1));  // Normalize bearing angle difference to [-pi, pi]

    // Calculate Kalman Gain using Jacobian
    Eigen::MatrixXd Hj = CalculateRadarJacobian(x_);  // Linearize measurement model

    Eigen::MatrixXd Ht = Hj.transpose();
    Eigen::MatrixXd S = Hj * P_ * Ht + R_radar_;  // Innovation covariance
    Eigen::MatrixXd K = P_ * Ht * S.inverse();    // Kalman gain

    // ==========================================
    // Update State and Covariance
    // ==========================================
    x_ = x_ + K * y;  // Correct state estimate

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * Hj) * P_;  // Update covariance

    // Ensure yaw angle stays in [-pi, pi] range
    NormalizeAngle(x_(3));
}
