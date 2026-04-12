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

  // Entry point that performs initialization, prediction, measurement update,
  // optional resampling, and weighted state extraction.
  void ProcessMeasurement(const MeasurementPackage& meas_package);

  // Propagate each particle with the CTRV process model and injected noise.
  void Prediction(double delta_t);

  // Weighted particle mean exposed to the rest of the highway harness.
  Eigen::VectorXd x_;

 private:
  struct Particle {
    Eigen::VectorXd x;
    double w;
  };

  // Reweight particles using the lidar likelihood p(z_k | x_k^i).
  void UpdateLidar(const MeasurementPackage& meas_package);

  // Reweight particles using the radar likelihood p(z_k | x_k^i).
  void UpdateRadar(const MeasurementPackage& meas_package);

  // Enforce a valid discrete probability distribution over particle weights.
  void NormalizeWeights();

  // Quantifies particle degeneracy to decide when to resample.
  double EffectiveSampleSize() const;

  // Low-variance resampling used when weight collapse is detected.
  void ResampleSystematic();

  // Recover the output state from the weighted particle cloud.
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
