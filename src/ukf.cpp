#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 1e-6
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // size of the state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // weights vector
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;

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

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);
  P_(0, 0) = 1.0;
  P_(1, 1) = 1.0;
  P_(2, 2) = 10.0;
  P_(3, 3) = 1.0;
  P_(4, 4) = M_PI / 8;

  // laser measurement function
  H_ = MatrixXd::Zero(2, 5);
  H_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // laser measurement covariance
  R_ = MatrixXd::Zero(2, 2);
  R_(0, 0) = std_laspx_ * std_laspx_;
  R_(1, 1) = std_laspy_ * std_laspy_;

  nis_laser_fs_.open("nis_laser.csv");
  nis_radar_fs_.open("nis_radar.csv");
}

UKF::~UKF() {
  nis_laser_fs_.close();
  nis_radar_fs_.close();
}

// Utility funtion to wrap a radian angle between -pi and +pi
float Wrap2pi(float rval) {
  while (rval > M_PI) {
    rval -= 2 * M_PI;
  }

  while (rval < -M_PI) {
    rval += 2 * M_PI;
  }
  return rval;
}

void UKF::SetProcessNoise(float a, float b) {
  std_a_ = a;
  std_yawdd_ = b;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (is_initialized_ == false) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       */
      float x = meas_package.raw_measurements_[0] *
                cos(meas_package.raw_measurements_[1]);
      float y = meas_package.raw_measurements_[0] *
                sin(meas_package.raw_measurements_[1]);
      x_ << x, y, 0, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
       * Initialize state.
       */
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
      if (fabs(x_(0)) < EPS && fabs(x_(1)) < EPS) {
        exit(0);
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) &&
      (use_radar_ == false)) {
    return;
  }
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) &&
      (use_laser_ == false)) {
    return;
  }

  // Predict
  double dt = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  // Radar update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);

    // Laser update
  } else {
    UpdateLidar(meas_package);
  }
  std::cout << "x_ = " << std::endl << x_ << std::endl;
  std::cout << "P_ = " << std::endl << P_ << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //
  // Generate Augmentation Sigma Points
  //
  // create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(7);
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(7, 7);
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  // create augmented mean state
  x_aug.head(5) = x_;
  // create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;
  // create square root matrix
  MatrixXd P_aug_root = P_aug.llt().matrixL();
  P_aug_root *= sqrt(lambda_ + n_aug_);
  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(1 + i) = x_aug + P_aug_root.col(i);
    Xsig_aug.col(8 + i) = x_aug - P_aug_root.col(i);
  }

  //
  // Predict Sigma Points
  //
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    if (fabs(Xsig_aug(4, i)) < 1e-3) {
      Xsig_pred_(0, i) =
          Xsig_aug(0, i) + Xsig_aug(2, i) * cos(Xsig_aug(3, i)) * delta_t +
          0.5 * delta_t * delta_t * cos(Xsig_aug(3, i)) * Xsig_aug(5, i);

      Xsig_pred_(1, i) =
          Xsig_aug(1, i) + Xsig_aug(2, i) * sin(Xsig_aug(3, i)) * delta_t +
          0.5 * delta_t * delta_t * sin(Xsig_aug(3, i)) * Xsig_aug(5, i);

      Xsig_pred_(2, i) = Xsig_aug(2, i) + delta_t * Xsig_aug(5, i);

      Xsig_pred_(3, i) =
          Xsig_aug(3, i) + 0.5 * delta_t * delta_t * Xsig_aug(6, i);

      Xsig_pred_(4, i) = Xsig_aug(4, i) + delta_t * Xsig_aug(6, i);
    } else {
      Xsig_pred_(0, i) =
          Xsig_aug(0, i) +
          Xsig_aug(2, i) / Xsig_aug(4, i) *
              (sin(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) -
               sin(Xsig_aug(3, i))) +
          0.5 * delta_t * delta_t * cos(Xsig_aug(3, i)) * Xsig_aug(5, i);

      Xsig_pred_(1, i) =
          Xsig_aug(1, i) +
          Xsig_aug(2, i) / Xsig_aug(4, i) *
              (cos(Xsig_aug(3, i)) -
               cos(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t)) +
          0.5 * delta_t * delta_t * sin(Xsig_aug(3, i)) * Xsig_aug(5, i);

      Xsig_pred_(2, i) = Xsig_aug(2, i) + delta_t * Xsig_aug(5, i);

      Xsig_pred_(3, i) = Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t +
                         0.5 * delta_t * delta_t * Xsig_aug(6, i);

      Xsig_pred_(4, i) = Xsig_aug(4, i) + delta_t * Xsig_aug(6, i);
    }
  }

  //
  // Predict mean and Covariance
  //
  // set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    MatrixXd T = Xsig_pred_.col(i) - x_;
    P_ = P_ + weights_(i) * T * T.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // setup new measurement
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

  // calculate the lidar NIS
  VectorXd z_diff = VectorXd::Zero(2);
  z_diff(0) = z(0) - x_(0);
  z_diff(1) = z(1) - x_(1);
  float e = z_diff.transpose() * S.inverse() * z_diff;
  nis_laser_fs_ << e << std::endl;

  // new state estimate
  MatrixXd K = P_ * Ht * S.inverse();
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_) * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  //
  // Predict RADAR Measurement
  //
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  // transform sigma points into measurement space
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    float px = Xsig_pred_(0, i);
    float py = Xsig_pred_(1, i);
    float v = Xsig_pred_(2, i);
    float yaw = Xsig_pred_(3, i);
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / Zsig(0, i);
  }
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // calculate innovation covariance matrix S
  S.fill(0.0);
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    MatrixXd T = Zsig.col(i) - z_pred;
    S = S + weights_(i) * T * T.transpose();
  }
  S = S + R;

  //
  // Update
  //
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], // rho in m
      meas_package.raw_measurements_[1],  // phi in rad
      meas_package.raw_measurements_[2];  // rho_dot in m/s
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {
    MatrixXd tx = Xsig_pred_.col(i) - x_;
    MatrixXd tz = Zsig.col(i) - z_pred;
    Tc = Tc + weights_(i) * tx * tz.transpose();
  }
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  z_diff(1) = Wrap2pi(z_diff(1)); // ensure wrap -pi to pi

  // calculate NIS
  float e = z_diff.transpose() * S.inverse() * z_diff;
  nis_radar_fs_ << e << std::endl;

  // update state mean and covariance matrix
  P_ = P_ - K * S * K.transpose();
  x_ = x_ + K * (z_diff);
}
