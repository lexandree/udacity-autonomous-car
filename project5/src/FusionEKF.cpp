#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

FusionEKF::~FusionEKF() {}

Eigen::VectorXd FusionEKF::GetState(){return ekf_.GetState();}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    // first measurement
    std::cout << "EKF: " << std::endl;
	previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
		ekf_.SetStateFromPolar(measurement_pack.raw_measurements_); // Convert radar from polar to cartesian coordinates 
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		ekf_.SetStateFromCartesian(measurement_pack.raw_measurements_); // Convert laser from cartesian to polar coordinates 
    // done initializing, no need to predict or update
    is_initialized_ = true;
    std::cout << "EKF: end" << std::endl;
    return;
  }
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  //Prediction
  ekf_.Predict(dt);
  //Update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    ekf_.UpdateRadar(measurement_pack.raw_measurements_);
  else
    ekf_.UpdateLaser(measurement_pack.raw_measurements_);

  // print the output
  std::cout << "x_ = " << ekf_.GetState() << std::endl;
  //std::cout << "P_ = " << ekf_.P_ << std::endl;
}
