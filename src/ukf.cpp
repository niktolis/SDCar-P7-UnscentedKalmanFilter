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
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.65;
  
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

  // Initialize covariance matrix.
  P_ << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;

  // Dimension of the state vector.
  n_x_ = static_cast<int> (x_.size());

  // Dimension of the augmented state vector.
  n_aug_ = static_cast<int>(x_.size()) + 2;

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

  // Process noise covariance matrix.
  Q_ = MatrixXd(2, 2);
  Q_ << std_a_ * std_a_, 0,
    0, std_yawdd_*std_yawdd_;

  // Radar measurement noise covariance matrix.
  R_r_ = MatrixXd(3, 3);
  R_r_ << std_radr_ * std_radr_, 0, 0,
    0, std_radphi_ * std_radphi_, 0,
    0, 0, std_radrd_ * std_radrd_;

  // Laser measurement noise covariance matrix.
  R_l_ = MatrixXd(2, 2);
  R_l_ << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;

  // force initialization.
  is_initialized_ = false;
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
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance.
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix.
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // fill augmented mean state vector.
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // fill augmented state covariance matrix.
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q_;

  // create square root matrix.
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points.
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  double delta_t_sq = delta_t * delta_t;

  // predict sigma points.
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);
    // predicted new yaw.
    double yaw_p = yaw + yawd * delta_t;

    // predicted state values.
    double px_p, py_p;

    if(yawd != 0) {
      px_p = px + v / yawd * (sin(yaw_p) - sin(yaw));
      py_p = py + v / yawd * (cos(yaw) - cos(yaw_p));
    } else {
      px_p = px + v * delta_t*cos(yaw);
      py_p = py + v * delta_t*sin(yaw);
    }

    double v_p = v;
    double yawd_p = yawd;

    // add noise.
    px_p = px_p + 0.5 * nu_a * delta_t_sq * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t_sq * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t_sq;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // predicted state mean.
  x_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix.
  P_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    // state difference.
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // normalize angle.
    x_diff(3) = this->NormalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

}

/*!
 * The common update part of the Unscented Kalman Filter regardless sensor type.
 *
 * @param {MeasurementPackage} meas_package
 * @param {MatrixXd} Zsig
 * @param {int} n_z 
 */

void UKF::UpdateCommon(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  // set measurement dimension, lidar can measure px, py.
  int n_z = 2;

  // create matrix for sigma points in measurment space.
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space.
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    Zsig(0, i) = Xsig_pred_(0, i);    // px
    Zsig(1, i) = Xsig_pred_(1, i);    // py
  }
  // mean predicted measurement.
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix.
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    // residual.
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix.
  S = S + R_l_;

  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference.
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K.
  MatrixXd K = Tc * S.inverse();

  // residual 
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;


  // update state mean and covariance matrix.
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate NIS.
  NIS_l_ = z_diff.transpose() * S.inverse() * z_diff;

  cout << "NIS_l_: " << NIS_r_ << endl << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // set measurement dimension, radar can measure r, phi, r_dot.
  int n_z = 3;

  // create matrix for sigma points in measurement space.
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space.
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model.
    Zsig(0, i) = sqrt(px * px + py * py);          // rho
    Zsig(1, i) = atan2(py, px);                    // phi
    if(Zsig(0, i) != 0) {
      Zsig(2, i) = (px * v1 + py * v2) / static_cast<double> (Zsig(0, i)); // rho dot
    } else {
      Zsig(2, i) = 0;
    }
    
  }

  // mean predicted measurement.
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix.
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    // residual.
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization.
    NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix.
  S = S + R_r_;

  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    z_diff(1) = NormalizeAngle(z_diff(1));

    // state difference.
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K.
  MatrixXd K = Tc * S.inverse();

  // residual 
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  // angle normalization.
  z_diff(1) = NormalizeAngle(z_diff(1));

  // update state mean and covariance matrix.
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate NIS.
  NIS_r_ = z_diff.transpose() * S.inverse() * z_diff;

  cout << "NIS_r_: " << NIS_r_ << endl << endl;
}

double UKF::NormalizeAngle(double phi) {
  
  phi = fmod(phi + M_PI, 2 * M_PI);
  if(phi < 0)
    phi += 2 * M_PI;

  return phi - M_PI;
}
