#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*! Initializes Unscented Kalman filter This is scaffolding, do not modify */
UKF::UKF() {
  /*! if this is false, laser measurements will be ignored (except during initialization) */
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during initialization)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Initialize covariance matrix.
  P_ << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;

  // Dimension of the state vector.
  n_x_ = x_.size();

  // Dimension of the augmented state vector.
  n_aug_ = x_.size() + 2;

  // Sigma point matrix.
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Spreading parameter.
  lambda_ = 3 - n_aug_;

  // Initialize weights.
  weights_ = VectorXd(2 * n_aug_ + 1);

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // Radar measurement noise covariance matrix.
  R_r_ = MatrixXd(3, 3);
  R_r_ << std_radr_ * std_radr_, 0, 0,
    0, std_radphi_ * std_radphi_, 0,
    0, 0, std_radrd_ * std_radrd_;

  // Laser measurement noise covariance matrix.
  R_l_ = MatrixXd(2, 2);
  R_l_ << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;
}

UKF::~UKF() {}


void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_) {

    cout << "UKF: " << endl;

    double px, py, v, vx, vy;

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rhod = meas_package.raw_measurements_[2];

      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rhod * cos(phi);
      vy = rhod * sin(phi);
      v = sqrt(vx * vx + vy * vy);

    } else if(meas_package.sensor_type_ == MeasurementPackage::LASER) {

      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      vx = 0;
      vy = 0;
      v = 0;
    }

    // set the state with the initial location and velocity.
    x_ << px, py, v, 0, 0;

    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update.
    is_initialized_ = true;

  } else {

    /*************************************************************************
    *  Prediction
    *************************************************************************/

    // new elapse time. Time is measured in seconds.
    double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;

    this->Prediction(dt);

    /*****************************************************************************
    *  Update
    ****************************************************************************/
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {

      // Radar updates
      this->UpdateRadar(meas_package);

    } else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {

      // Laser updates
      this->UpdateLidar(meas_package);
    }

  }
  
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // create augmented mean vector.
  VectorXd x_aug = VectorXd(n_aug_)
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

double UKF::NormalizeAngle(double phi) {
  
  phi = fmod(phi + M_PI, 2 * M_PI);
  if(phi < 0)
    phi += 2 * M_PI;

  return phi - M_PI;
}
