#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  //  * the estimation vector size should not be zero
  if (estimations.size()==0){
    std::cout << "estimations size is zero" << std::endl;
    return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size()!=ground_truth.size()){
    std::cout << "estimations size is invalid" << std::endl;
    return rmse;
  }
  VectorXd s(4);
  for (unsigned int i=0; i < estimations.size(); ++i) {
    s = estimations[i] - ground_truth[i];
    s = s.array() * s.array();
    rmse += s;
  }

  rmse = rmse.array() / estimations.size();
  rmse = rmse.array().sqrt();
  std::cout << rmse << std::endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float pxy = px*px + py*py;
  // check division by zero
  if (fabs(pxy) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }
  float k1 = sqrt(pxy);
  float k2 = k1*pxy;
  float k3 = px*vy - py*vx;
  Hj << px/k1, py/k1, 0, 0,
        -py/pxy, px/pxy, 0, 0,
        -py*k3/k2, px*k3/k2, px/k1, py/k1;

  return Hj;
}
