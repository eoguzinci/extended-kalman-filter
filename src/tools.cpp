#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.empty())
	   throw "Cannot operate on empty vector!";
  if( estimations.size()!=ground_truth.size())
     throw "Matrices must be in the same size!";
        
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
	}

	//calculate the mean
	// ... your code here
    rmse = rmse/estimations.size();
	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	// PRE-COMPUTING
  float coef1 = px*px+py*py;
  float coef2 = pow(coef1,1.5);
  float coef3 = sqrt(coef1);

  //check division by zero
  if (fabs(coef1) < 1e-4){
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return Hj;
  }

	//compute the Jacobian matrix
	Hj(0,0) = px/coef3;
	Hj(0,1) = py/coef3;
	Hj(0,2) = 0;
	Hj(0,3) = 0;
	Hj(1,0) = -py/(coef1);
	Hj(1,1) = px/(coef1);
	Hj(1,2) = 0;
	Hj(1,3) = 0;
	Hj(2,0) = py*(vx*py-vy*px)/coef2;
	Hj(2,1) = px*(vy*px-vx*py)/coef2;
	Hj(2,2) = px/coef3;
	Hj(2,3) = py/coef3;

	return Hj;
}
