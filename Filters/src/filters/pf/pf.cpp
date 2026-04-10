#include "pf.h"

#include <algorithm>
#include <cmath>

using Eigen::VectorXd;

PF::PF()
    : is_initialized_(false),
      previous_timestamp_(0),
      n_x_(5),
  n_particles_(1200),
  std_a_(0.35),
  std_yawdd_(0.25),
      std_laspx_(0.15),
      std_laspy_(0.15),
      std_radr_(0.3),
      std_radphi_(0.03),
      std_radrd_(0.3),
      rng_(std::random_device{}()) {
  x_ = VectorXd::Zero(n_x_);
  particles_.resize(n_particles_);
  for (int i = 0; i < n_particles_; ++i) {
    particles_[i].x = VectorXd::Zero(n_x_);
    particles_[i].w = 1.0 / n_particles_;
  }
}

PF::~PF() {}

double PF::NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void PF::ProcessMeasurement(const MeasurementPackage& meas_package) {
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0.0;
      x_(3) = 0.0;
      x_(4) = 0.0;
    } else {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rhod = meas_package.raw_measurements_(2);
      x_(0) = rho * std::cos(phi);
      x_(1) = rho * std::sin(phi);
      x_(2) = std::fabs(rhod);
      x_(3) = phi;
      x_(4) = 0.0;
    }

    std::normal_distribution<double> n_px(0.0, 0.25);
    std::normal_distribution<double> n_py(0.0, 0.25);
    std::normal_distribution<double> n_v(0.0, 0.6);
    std::normal_distribution<double> n_yaw(0.0, 0.8);
    std::normal_distribution<double> n_yawd(0.0, 0.2);

    for (int i = 0; i < n_particles_; ++i) {
      particles_[i].x(0) = x_(0) + n_px(rng_);
      particles_[i].x(1) = x_(1) + n_py(rng_);
      particles_[i].x(2) = x_(2) + n_v(rng_);
      particles_[i].x(3) = x_(3) + n_yaw(rng_);
      particles_[i].x(4) = x_(4) + n_yawd(rng_);
      particles_[i].w = 1.0 / n_particles_;
    }

    ComputeStateMean();
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  const double dt =
      (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else {
    UpdateRadar(meas_package);
  }

  NormalizeWeights();
  if (EffectiveSampleSize() < 0.55 * n_particles_) {
    ResampleSystematic();
  }
  ComputeStateMean();
}

void PF::Prediction(double delta_t) {
  std::normal_distribution<double> n_a(0.0, std_a_);
  std::normal_distribution<double> n_yawdd(0.0, std_yawdd_);

  for (int i = 0; i < n_particles_; ++i) {
    const double px = particles_[i].x(0);
    const double py = particles_[i].x(1);
    const double v = particles_[i].x(2);
    const double yaw = particles_[i].x(3);
    const double yawd = particles_[i].x(4);

    const double nu_a = n_a(rng_);
    const double nu_yawdd = n_yawdd(rng_);

    double px_p;
    double py_p;
    if (std::fabs(yawd) > 1e-4) {
      px_p = px + (v / yawd) *
                      (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
      py_p = py + (v / yawd) *
                      (-std::cos(yaw + yawd * delta_t) + std::cos(yaw));
    } else {
      px_p = px + v * std::cos(yaw) * delta_t;
      py_p = py + v * std::sin(yaw) * delta_t;
    }

    const double dt2 = delta_t * delta_t;

    particles_[i].x(0) = px_p + 0.5 * dt2 * std::cos(yaw) * nu_a;
    particles_[i].x(1) = py_p + 0.5 * dt2 * std::sin(yaw) * nu_a;
    particles_[i].x(2) = std::max(0.0, v + delta_t * nu_a);
    particles_[i].x(3) = NormalizeAngle(yaw + yawd * delta_t +
                                        0.5 * dt2 * nu_yawdd);
    particles_[i].x(4) = yawd + delta_t * nu_yawdd;
  }
}

void PF::UpdateLidar(const MeasurementPackage& meas_package) {
  const double var_x = std_laspx_ * std_laspx_;
  const double var_y = std_laspy_ * std_laspy_;
  const double norm = 1.0 / (2.0 * M_PI * std_laspx_ * std_laspy_);

  const double zpx = meas_package.raw_measurements_(0);
  const double zpy = meas_package.raw_measurements_(1);

  for (int i = 0; i < n_particles_; ++i) {
    const double dx = zpx - particles_[i].x(0);
    const double dy = zpy - particles_[i].x(1);
    const double expo = -0.5 * ((dx * dx) / var_x + (dy * dy) / var_y);
    particles_[i].w *= norm * std::exp(expo);
  }
}

void PF::UpdateRadar(const MeasurementPackage& meas_package) {
  const double var_r = std_radr_ * std_radr_;
  const double var_phi = std_radphi_ * std_radphi_;
  const double var_rd = std_radrd_ * std_radrd_;
  const double norm =
      1.0 / ((2.0 * M_PI) * std::sqrt(var_r * var_phi * var_rd));

  const double zr = meas_package.raw_measurements_(0);
  const double zphi = meas_package.raw_measurements_(1);
  const double zrd = meas_package.raw_measurements_(2);

  for (int i = 0; i < n_particles_; ++i) {
    const double px = particles_[i].x(0);
    const double py = particles_[i].x(1);
    const double v = particles_[i].x(2);
    const double yaw = particles_[i].x(3);

    const double vx = v * std::cos(yaw);
    const double vy = v * std::sin(yaw);

    const double rho = std::max(1e-6, std::sqrt(px * px + py * py));
    const double phi = std::atan2(py, px);
    const double rhod = (px * vx + py * vy) / rho;

    const double dr = zr - rho;
    const double dphi = NormalizeAngle(zphi - phi);
    const double drd = zrd - rhod;

    const double expo =
        -0.5 * ((dr * dr) / var_r + (dphi * dphi) / var_phi +
                (drd * drd) / var_rd);
    particles_[i].w *= norm * std::exp(expo);
  }
}

void PF::NormalizeWeights() {
  double s = 0.0;
  for (int i = 0; i < n_particles_; ++i) {
    particles_[i].w = std::max(1e-300, particles_[i].w);
    s += particles_[i].w;
  }
  if (s <= 0.0) {
    const double w = 1.0 / n_particles_;
    for (int i = 0; i < n_particles_; ++i) particles_[i].w = w;
    return;
  }
  for (int i = 0; i < n_particles_; ++i) particles_[i].w /= s;
}

double PF::EffectiveSampleSize() const {
  double sum_sq = 0.0;
  for (int i = 0; i < n_particles_; ++i) {
    sum_sq += particles_[i].w * particles_[i].w;
  }
  return (sum_sq > 0.0) ? 1.0 / sum_sq : 0.0;
}

void PF::ResampleSystematic() {
  std::vector<Particle> new_particles(n_particles_);

  std::vector<double> cdf(n_particles_);
  cdf[0] = particles_[0].w;
  for (int i = 1; i < n_particles_; ++i) cdf[i] = cdf[i - 1] + particles_[i].w;

  std::uniform_real_distribution<double> unif(0.0, 1.0 / n_particles_);
  const double u0 = unif(rng_);

  int idx = 0;
  for (int m = 0; m < n_particles_; ++m) {
    const double u = u0 + (static_cast<double>(m) / n_particles_);
    while (idx < n_particles_ - 1 && u > cdf[idx]) ++idx;
    new_particles[m] = particles_[idx];
    new_particles[m].w = 1.0 / n_particles_;
  }

  // Light roughening mitigates sample impoverishment after resampling.
  std::normal_distribution<double> n_px(0.0, 0.03);
  std::normal_distribution<double> n_py(0.0, 0.03);
  std::normal_distribution<double> n_v(0.0, 0.06);
  std::normal_distribution<double> n_yaw(0.0, 0.01);
  std::normal_distribution<double> n_yawd(0.0, 0.01);
  for (int i = 0; i < n_particles_; ++i) {
    new_particles[i].x(0) += n_px(rng_);
    new_particles[i].x(1) += n_py(rng_);
    new_particles[i].x(2) = std::max(0.0, new_particles[i].x(2) + n_v(rng_));
    new_particles[i].x(3) = NormalizeAngle(new_particles[i].x(3) + n_yaw(rng_));
    new_particles[i].x(4) += n_yawd(rng_);
  }

  particles_.swap(new_particles);
}

void PF::ComputeStateMean() {
  x_.setZero();
  double s = 0.0;

  for (int i = 0; i < n_particles_; ++i) {
    x_(0) += particles_[i].w * particles_[i].x(0);
    x_(1) += particles_[i].w * particles_[i].x(1);
    x_(2) += particles_[i].w * particles_[i].x(2);
    s += particles_[i].w;
  }

  double sin_yaw = 0.0;
  double cos_yaw = 0.0;
  for (int i = 0; i < n_particles_; ++i) {
    sin_yaw += particles_[i].w * std::sin(particles_[i].x(3));
    cos_yaw += particles_[i].w * std::cos(particles_[i].x(3));
    x_(4) += particles_[i].w * particles_[i].x(4);
  }

  if (s > 0.0) {
    x_(0) /= s;
    x_(1) /= s;
    x_(2) /= s;
    x_(4) /= s;
  }
  x_(3) = std::atan2(sin_yaw, cos_yaw);
}
