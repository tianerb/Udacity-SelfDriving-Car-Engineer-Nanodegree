#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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
  is_initialized_ = false;
  P_ << 1, 0, 0, 0, 0,
  0, 1, 0, 0, 0,
  0, 0, 1, 0, 0,
  0, 0, 0, 1, 0,
  0, 0, 0, 0, 1;
    
  time_us_ = 0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  weights_ = VectorXd(2*n_aug_+1);
  // set weights
  const double weight_0_ = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0_;
  const double weight_ = 0.5/(n_aug_+lambda_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = weight_;
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      const double rho = meas_package.raw_measurements_[0];
      const double phi = meas_package.raw_measurements_[1];

      x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0],
      meas_package.raw_measurements_[1], 0, 0, 0;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  }

  const double delta_t_ = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t_);

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);

  //create augmented mean state
  x_aug_.head(n_x_) = x_;
  x_aug_(n_x_) = 0;
  x_aug_(n_x_+1) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_, n_x_) = std_a_*std_a_;
  P_aug_(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  //create square root matrix
  const MatrixXd L_ = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i=0; i<n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * L_.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L_.col(i);
  }

  //predict sigma points
  for(int i=0; i<2*n_aug_+1; i++){
    const double v = Xsig_aug_(2,i);
    const double yaw = Xsig_aug_(3,i);
    const double yawd = Xsig_aug_(4,i);
    const double nu_a = Xsig_aug_(5,i);
    const double nu_yawdd = Xsig_aug_(6,i);
    VectorXd x_in = MatrixXd(n_x_,1);
    if(fabs(yawd) < 0.001){
    x_in << v * cos(yaw) * delta_t + 0.5 * delta_t*delta_t * cos(yaw) * nu_a,
        v * sin(yaw) * delta_t + 0.5 * delta_t*delta_t * sin(yaw) * nu_a,
        0 + delta_t * nu_a,
        0 + 0.5 * delta_t*delta_t * nu_yawdd,
        0 + delta_t * nu_yawdd;
    }
    else{
        x_in << v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw)) + 0.5 * delta_t*delta_t * cos(yaw) * nu_a,
        v / yawd * (-cos(yaw + yawd*delta_t) + cos(yaw)) + 0.5 * delta_t*delta_t * sin(yaw) * nu_a,
        0 + delta_t * nu_a,
        yawd * delta_t + 0.5 * delta_t*delta_t * nu_yawdd,
        0 + delta_t * nu_yawdd;
    }
    x_in += Xsig_aug_.col(i).head(n_x_);
    Xsig_pred_.col(i) = x_in;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(x_diff(3));
    
    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
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
    MatrixXd H_ = MatrixXd(2, n_x_);
    H_ << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0;
    MatrixXd R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;
    const VectorXd z = meas_package.raw_measurements_;
    const VectorXd z_pred = H_ * x_;
    const VectorXd y = z - z_pred;
    const MatrixXd PHt = P_ * H_.transpose();
    const MatrixXd S = H_ * PHt + R_laser_;
    const MatrixXd K = PHt * S.inverse();
    
    x_ += K * y;
    P_ -= K * H_ * P_;
    
    NIS_laser_ = y.transpose() * S.inverse() * y;
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
  //create matrix for sigma points in measurement space
  int n_z_ = 3;
  MatrixXd Zsig_ = MatrixXd(n_z_, 2*n_aug_+1);

  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 simga points

    // extract values for better readibility
    const double p_x = Xsig_pred_(0,i);
    const double p_y = Xsig_pred_(1,i);
    const double v  = Xsig_pred_(2,i);
    const double yaw = Xsig_pred_(3,i);

    const double v1 = cos(yaw)*v;
    const double v2 = sin(yaw)*v;

    // measurement model
    Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig_(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred += weights_(i) * Zsig_.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    
    //angle normalization
    NormalizeAngle(z_diff(1));
    
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R_radar_ = MatrixXd(n_z_,n_z_);
  R_radar_ << std_radr_*std_radr_, 0, 0,
  0, std_radphi_*std_radphi_, 0,
  0, 0,std_radrd_*std_radrd_;
  S += R_radar_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 simga points
    
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    //angle normalization
    NormalizeAngle(z_diff(1));
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(x_diff(3));
    
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
    
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;

  //angle normalization
  NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
  
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
