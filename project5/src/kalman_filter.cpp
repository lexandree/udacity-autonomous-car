#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

KalmanFilter::KalmanFilter() {
  // initializing matrices
  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  //measurement matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
  // the initial cartesian and polar states
  x_ = VectorXd(4);
  x_ << 0, 0, 0, 0;
  polar_ = VectorXd(3);
  polar_ << 0, 0, 0;
  // set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;
  Qn_ = MatrixXd(2, 2);
  Qn_ << noise_ax, 0,
        0, noise_ay;
  std::cout << "KalmanFilter created" << std::endl;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::SetStateFromCartesian(const VectorXd &x) {
  x_ << x(0), //px
		x(1), //py
		0, //init vx
		0; //init vy
  if (x.size() == 4){
    x_ << 	x(0), //px
			x(1), //py
			x(2), //vx
			x(3); //vy
  }
  // Convert from cartesian to polar coordinates
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / std::max(rho, 0.0001);
  polar_ << rho, phi, rho_dot;
}

void KalmanFilter::SetStateFromPolar(const VectorXd &polar) {
  polar_ << polar(0), //rho
			polar(1), //phi
			polar(2); //rho_dot
  // Convert from polar to cartesian coordinates
  float px = polar(0) * cos(polar(1)); //rho * cos(phi)
  float py = polar(0) * sin(polar(1)); //rho * sin(phi)
  float vx = polar(2) * cos(polar(1)); //rho_dot * cos(phi)
  float vy = polar(2) * sin(polar(1)); //rho_dot * sin(phi)
  x_ << px, py, vx, vy;
}

void KalmanFilter::Predict(const float dt) {
  // 1. Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;
  // 2. Set the process covariance matrix Q
  MatrixXd G_;
  G_ = MatrixXd(4, 2);
  G_ << (dt*dt)/2, 0,
        0, (dt*dt)/2,
        dt, 0,
        0, dt;
  MatrixXd Q_;
  Q_ = MatrixXd(4, 4);
  Q_ = G_ * Qn_ * G_.transpose();
  // compute prediction
  SetStateFromCartesian(F_ * x_); //x_ = F_ * x_
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& y, const MatrixXd &H_, const MatrixXd &R_) {

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;

	// Compute Kalman gain
	MatrixXd K = P_ * Ht * S.inverse();

	// Update estimate
	SetStateFromCartesian(x_ + K * y); // x_ = x_ + K * y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateLaser(const VectorXd &z) {

  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  
  Update(y, H_laser_, R_laser_);
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {
  
  VectorXd y = z - polar_;

  // Normalize angle
  //while (y(1) > M_PI) y(1) -= 2 * M_PI;
  //while (y(1) < -M_PI) y(1) += 2 * M_PI;
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= 2 * M_PI;
    } else {
      y(1) += 2 * M_PI;
    }
  }
  
  Update(y, tools.CalculateJacobian(x_), R_radar_);
}

const VectorXd &KalmanFilter::GetState() const{return x_;}

