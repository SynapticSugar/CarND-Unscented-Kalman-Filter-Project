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
    * Calculate the RMSE here.
    */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0) {
    cout << "the estimation vector size should not be zero";
    return rmse;
  }

  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()) {
    cout << "the estimation vector size should equal ground truth vector size";
    return rmse;
  }

  // accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd a(4);
    a = estimations[i] - ground_truth[i];
    rmse = rmse.array() + a.array() * a.array();
  }

  // calculate the mean
  rmse = rmse.array() / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}