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
  rmse << 0,0,0,0;

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){

    VectorXd estimation = estimations[i];
    VectorXd ground_th = ground_truth[i];
    if (estimation.size() == 0 ){
      cout << "estimations vector should not be empty " << endl;
      continue;
    }

     if (estimation.size() != ground_th.size() ){
      cout << "estimation and ground thruth vector size should match " << endl;
      continue;
    }

    VectorXd deviation= estimation - ground_th; // deviation in each feature
    VectorXd square_deviation = deviation.array() * deviation.array(); // sqare of deviation
    rmse += square_deviation;  // accumulating square of deviations

    //cout<<"rmse"<< endl << rmse << endl;
  } // end of for

  
  
  //calculate the mean values of each feature
  rmse = rmse/estimations.size();
  //calculate the squared root of mean
  rmse=rmse.array().sqrt();

  /**
  if (rmse(0) > 10.0 || rmse(1) > 10.0 || rmse(2) > 10.0 || rmse(3) > 10.0){
    cout << "exiting as values are very high " << rmse << endl;
    exit(1);
  }
  */
  
  //return the result
  return rmse;
  
}