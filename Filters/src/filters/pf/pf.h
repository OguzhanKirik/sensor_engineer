#ifndef PF_H_
#define PF_H_

#include <random>
#include <vector>

#include <Eigen/Dense>

#include "../../measurement_package.h"

class PF {
 public:
  PF();
  virtual ~PF();

  void ProcessMeasurement(const MeasurementPackage& meas_package);
  void Prediction(double delta_t);

  Eigen::VectorXd x_;

 private:
  struct Particle {
    Eigen::VectorXd x;
    double w;
  };

  void UpdateLidar(const MeasurementPackage& meas_package);
  void UpdateRadar(const MeasurementPackage& meas_package);
  void NormalizeWeights();
  double EffectiveSampleSize() const;
  void ResampleSystematic();
  void ComputeStateMean();

  static double NormalizeAngle(double angle);

  bool is_initialized_;
  long long previous_timestamp_;

  int n_x_;
  int n_particles_;

  double std_a_;
  double std_yawdd_;
  double std_laspx_;
  double std_laspy_;
  double std_radr_;
  double std_radphi_;
  double std_radrd_;

  std::vector<Particle> particles_;
  std::mt19937 rng_;
};

#endif
