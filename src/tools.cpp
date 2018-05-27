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
  rmse<<0,0,0,0;
  if (estimations.size()!=ground_truth.size())
  {
	  cout<<"Sizes are not equal"<<endl;
	  return rmse;
  }
  if (estimations.size()==0 ||ground_truth.size()==0)
  {
	  cout<<"RMSE Calculation size is 0"<<endl;
	  return rmse;
  }
  for(unsigned int i=0;i<estimations.size();++i)
  {
	  VectorXd diff = estimations[i]-ground_truth[i];
	  diff = diff.array()*diff.array();
	  rmse +=diff;
  }
  unsigned int n = estimations.size();
  rmse =rmse/n;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  float px=x_state(0);
  float py=x_state(1);
  float vx=x_state(2);
  float vy=x_state(3);
  if (fabs(px) < 0.0001 and fabs(py) < 0.0001){
	  px = 0.0001;
	  py = 0.0001;
  }
  
  MatrixXd Hj(3,4);
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  if(fabs(c1) < 0.0001){
		c1 =0.0001;
	}
  Hj << (px/c2), (py/c2), 0, 0,
		 -(py/c1), (px/c1), 0, 0,
		 py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;   
  return Hj;
}
