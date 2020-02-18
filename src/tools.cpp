#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  vector<VectorXd>::const_iterator it_e = estimations.begin();
  int size = ground_truth[0].size();
  VectorXd sum_error = VectorXd::Zero(size);
  
  for(vector<VectorXd>::const_iterator it_g = ground_truth.begin(); 
      it_g != ground_truth.end(); ++it_g,++it_e) {
    VectorXd resuidal = *it_g - *it_e;
    VectorXd resuidal_sq = resuidal.array().square();
    sum_error += resuidal_sq;
  }

  VectorXd mse = sum_error/ground_truth.size();
  VectorXd rmse = mse.cwiseSqrt();
  std:: cout <<"RMSE : \n" << rmse<< std::endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

 float rho_sq = pow(px,2)+pow(py,2);
 float rho = pow(rho_sq, 0.5);
 float rho_3_2 = rho_sq*rho;
 float vx_py = vx*py;
 float vy_px = vy*px;
 
 if (rho_sq == 0) {
     std::cout << "Error: Division by zero";
     return Hj;
 } else {
    Hj = MatrixXd::Zero(3,4);
    Hj(0,0) =px/rho;
    Hj(0,1) =py/rho;
    Hj(1,0) =-1*py/rho_sq;
    Hj(1,1) = px/rho_sq;
    Hj(2,0) = py*(vx_py-vy_px)/rho_3_2;
    Hj(2,1) = px*(vy_px-vx_py)/rho_3_2;
    Hj(2,2) = Hj(0,0);
    Hj(2,3) = Hj(0,1);
 }
   return Hj;
  
}
