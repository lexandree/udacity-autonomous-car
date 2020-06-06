#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"


class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Updates state from cartesian vector, compute polar state
   * @param x The new cartesian state  px, py, vx, vy
   */
  void SetStateFromCartesian(const Eigen::VectorXd &x);
   
  /**
   * Updates state from polar vector, compute cartesian state
   * @param polar The new polar state  rho, phi, rho_dot
   */
  void SetStateFromPolar(const Eigen::VectorXd &polar);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const float dt);
  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateLaser(const Eigen::VectorXd &z);
  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateRadar(const Eigen::VectorXd &z);
  //Eigen::VectorXd GetState();
  const Eigen::VectorXd &GetState() const;

 private:
  // polar state vector
  Eigen::VectorXd polar_;
  // cartesian state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  Tools tools;

  void Update(const Eigen::VectorXd& y, const Eigen::MatrixXd &H_, const Eigen::MatrixXd &R_);
  // state transition matrix
  Eigen::MatrixXd F_;
  // acceleration noise 
  Eigen::MatrixXd Qn_;
  //measurement covariance matrix - laser
  Eigen::MatrixXd R_laser_;
  //measurement covariance matrix - radar
  Eigen::MatrixXd R_radar_;
  // Lidar - measurement matrix
  Eigen::MatrixXd H_laser_;
};

#endif // KALMAN_FILTER_H_
