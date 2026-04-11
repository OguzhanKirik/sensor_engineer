#include "ukf.h"
#include <Eigen/Dense>
#include <cassert>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// UKF forward pass for the 5D CTRV highway tracking problem.

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

   /*
   Initialization of the member properties
   */
  is_initialized_ = false;
  time_us_ = 0;

  n_x_ = 5; // State dimension
  
  n_aug_ = 7; // state dimnesion + process noises
  
  // Spreading parameter for UKF
  lambda_ = 3 - n_aug_; 
  
  weights_ = VectorXd(2 * n_aug_ + 1); //weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); //predicted sigma points matrix
  last_x_pred_ = VectorXd::Zero(n_x_);
  last_P_pred_ = MatrixXd::Identity(n_x_, n_x_);
  last_P_cross_ = MatrixXd::Zero(n_x_, n_x_);
  

}

UKF::~UKF() {}

void UKF::ClearStepHistory() { step_history_.clear(); }

const std::vector<UKFRTSStepData>& UKF::GetStepHistory() const {
  return step_history_;
}

void UKF::NormalizeAngle(double& angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  assert(n_x_ ==5);
  assert(n_aug_ ==7);
  assert(lambda_ == 3 - n_aug_);
  assert(weights_.size() == 2 * n_aug_ + 1);


  if(is_initialized_ == false){
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      assert(std::isfinite(meas_package.raw_measurements_(0)));
      assert(std::isfinite(meas_package.raw_measurements_(1)));

      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0,0,0;
      P_ << std_laspx_*std_laspx_,0,0,0,0,
            0,std_laspy_*std_laspy_,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      assert(std::isfinite(meas_package.raw_measurements_(0)));
      assert(std::isfinite(meas_package.raw_measurements_(1)));
      assert(std::isfinite(meas_package.raw_measurements_(2)));
      assert(meas_package.raw_measurements_(0) >= 0.0);
      
      double rho = meas_package.raw_measurements_(0); // distance to the object
      double phi = meas_package.raw_measurements_(1); // angle to the object
      double rho_dot = meas_package.raw_measurements_(2); // radial velocity
      x_ << rho*cos(phi), rho*sin(phi), 0,0,0;
      P_ << std_radr_*std_radr_,0,0,0,0,
            0,std_radr_*std_radr_,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;
    }

    UKFRTSStepData init_step;
    init_step.timestamp = time_us_;
    init_step.is_initialization = true;
    init_step.x_filtered = x_;
    init_step.P_filtered = P_;
    init_step.x_predicted = x_;
    init_step.P_predicted = P_;
    init_step.P_cross = P_;
    step_history_.push_back(init_step);
    return; // Return after initialization, don't process first measurement
  }
  
  // Compute time delta
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  
  // Prediction step
  Prediction(dt); 
  
  // Update step
  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      UpdateLidar(meas_package);
  }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      UpdateRadar(meas_package );
  }

  // Cache the posterior, prediction, and cross-covariance needed by the
  // backward smoothers.
  UKFRTSStepData step;
  step.timestamp = meas_package.timestamp_;
  step.is_initialization = false;
  step.x_filtered = x_;
  step.P_filtered = P_;
  step.x_predicted = last_x_pred_;
  step.P_predicted = last_P_pred_;
  step.P_cross = last_P_cross_;
  step_history_.push_back(step);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  
  //assert(delta_t > 0.0);
  assert(x_.size() == n_x_);
  assert(P_.rows() == n_x_ && P_.cols() == n_x_);
  assert(weights_.size() == 2 * n_aug_ + 1);
  assert(lambda_ + n_aug_ > 0.0);

  // Preserve x_{k|k} and P_{k|k}; the smoother cross-covariance couples these
  // terms to the one-step prediction at k+1.
  const VectorXd x_filtered_before = x_;
  const MatrixXd P_filtered_before = P_;

  // Build the augmented Gaussian that explicitly carries process noise terms.
  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented convariace
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0); //P_aug.setZero()
  P_aug.topLeftCorner(5,5) = P_;    
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;


  //create square root matrix,the Cholesky factor
  MatrixXd L = P_aug.llt().matrixL(); 

  P_aug = L * L.transpose();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  assert(Xsig_aug.rows() == n_aug_ && Xsig_aug.cols() == 2 * n_aug_ + 1);
  assert(Xsig_pred_.allFinite());
  
  
  // Propagate each sigma point through the nonlinear CTRV process model.
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability 
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    assert(std::isfinite(p_x));
    assert(std::isfinite(p_y));
    assert(std::isfinite(v));
    assert(std::isfinite(yaw));
    assert(std::isfinite(yawd));
    assert(std::isfinite(nu_a));
    assert(std::isfinite(nu_yawdd));

    //avoid division by zero
    if (fabs(yawd) > 0.001){
      px_p = p_x + v /yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }else{
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }
      double v_p = v;
      double yaw_p = yaw + yawd * delta_t;
      double yawd_p = yawd;

      px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
      py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
      v_p += nu_a * delta_t;

      yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
      yawd_p += nu_yawdd * delta_t;

      assert(std::isfinite(px_p));
      assert(std::isfinite(py_p));
      assert(std::isfinite(v_p));
      assert(std::isfinite(yaw_p));
      assert(std::isfinite(yawd_p));

      Xsig_pred_(0, i) = px_p;
      Xsig_pred_(1, i) = py_p;
      Xsig_pred_(2, i) = v_p;
      Xsig_pred_(3, i) = yaw_p;
      Xsig_pred_(4, i) = yawd_p;

      assert(Xsig_pred_.allFinite());
  }

  assert(weights_.size() == 2 * n_aug_ + 1);
  assert(std::isfinite(weights_.sum()));
  assert(fabs(weights_.sum() - 1.0) < 1e-6);

  // Recover the predicted mean and covariance from the propagated sigma set.
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  assert(x_.size() == n_x_);
  assert(x_.allFinite());

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

  assert(P_.rows() == n_x_ && P_.cols() == n_x_);
  assert(P_.allFinite());
  assert(P_.isApprox(P_.transpose()));
  assert(P_(0,0) >= 0.0);
  assert(P_(1,1) >= 0.0);
  assert(P_(2,2) >= 0.0);
  assert(P_(3,3) >= 0.0);
  assert(P_(4,4) >= 0.0);

  last_x_pred_ = x_;
  last_P_pred_ = P_;

  // Cross-covariance needed by the unscented RTS and fixed-lag smoothers.
  MatrixXd Xsig_state = Xsig_aug.topRows(n_x_);
  last_P_cross_ = MatrixXd::Zero(n_x_, n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_state.col(i) - x_filtered_before;
    VectorXd x_pred_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));
    NormalizeAngle(x_pred_diff(3));
    last_P_cross_ += weights_(i) * x_diff * x_pred_diff.transpose();
  }


}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // Transform predicted signa points into measuremtn space
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ +1);
  for(int i=0;i<2 * n_aug_ + 1;i++){
    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);
  }
  assert(Zsig.rows() == n_z && Zsig.cols() == 2 * n_aug_ + 1);
  assert(Zsig.allFinite());

  // compute predicted measurement space // mean predicted postioin
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i=0;i<2 * n_aug_ + 1;i++){
    z_pred += weights_(i) * Zsig.col(i);
  }
  assert(z_pred.size() == n_z);
  assert(z_pred.allFinite());

  //Compute measurement covariance // difference between mean predciiotn and sigma points
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      S += weights_(i) * z_diff * z_diff.transpose();
  }

  //Measurement noise covariance
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;

  S += R;


  //Compute cross-correlation matrix
  //State covariance P_: 
  //how uncertain each state variable is
  //how state variables are correlated with each other
  //Measurement covariance S
  //how uncertain the predicted measurement is
  //how measurement components are correlated with each other
  //Cross-correlation matrix Tc :how state errors are 
  //related to measurement errors

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //Compute Kalman gain
  MatrixXd K = Tc * S.inverse();

  //Use the real lidar measurement
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;

  //Update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  NormalizeAngle(x_(3));

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

   int n_z = 3;
   MatrixXd Zsig= MatrixXd(n_z, 2 * n_aug_ + 1);

   // Tranfer predicted sigma points in measuremnet space
  for(int i=0;i< 2 *n_aug_ + 1;i++){
    double p_x  = Xsig_pred_(0, i);
    double p_y  = Xsig_pred_(1, i);
    double v    = Xsig_pred_(2, i);
    double yaw  = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    double rho = sqrt(p_x * p_x + p_y * p_y);
    if (rho < 1e-6) {
        rho = 1e-6;
    }

    Zsig(0, i) = rho;
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2) / rho;
  }

  //compute mean predicted measuremnt space
  VectorXd z_pred(n_z); z_pred.fill(0.0);
  for(int i=0;i< 2 *n_aug_ + 1;i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  assert(z_pred.size() == n_z);
  assert(z_pred.allFinite());

  //Compute the measuremetnt covariance  S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;

      while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

      S += weights_(i) * z_diff * z_diff.transpose();
  }


  //Measurement noise covariance
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
    0, std_radphi_ * std_radphi_, 0,
    0, 0, std_radrd_ * std_radrd_;

  S += R;

  // compute the cross corrleations matrix T
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

    //Compute Kalman gain
    MatrixXd K = Tc * S.inverse();

    // actual radar measurement
    VectorXd z = meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;


    //Update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    NormalizeAngle(x_(3));

}
