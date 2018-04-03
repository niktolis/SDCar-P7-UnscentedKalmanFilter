#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {

  product_sum_ = VectorXd(4);
  product_sum_.fill(0.0);
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse;

  // check the validity of the following inputs:
  // * the estimation vector size should not be zero
  // * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "CalculateRMSE() - Error - Invalid estimation or ground truth data!" << endl;
  } else {
    unsigned int n = static_cast<unsigned int>(estimations.size());

    VectorXd residual = estimations[n - 1] - ground_truth[n - 1];

    // coefficient-wise multiplication.
    residual = residual.array()*residual.array();

    // accumulate squared residuals.
    product_sum_ += residual;
  }

  // calculate the mean.
  rmse = product_sum_ / static_cast<int>(estimations.size());

  // calculate the squared root.
  rmse = rmse.array().sqrt();

  // return the result.
  return rmse;
}