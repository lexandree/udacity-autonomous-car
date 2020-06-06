#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"

class FusionEKF {
 public:
  FusionEKF();
  virtual ~FusionEKF();
  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  Eigen::VectorXd GetState();

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  // previous timestamp
  long long previous_timestamp_;
  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;
};

#endif // FusionEKF_H_
