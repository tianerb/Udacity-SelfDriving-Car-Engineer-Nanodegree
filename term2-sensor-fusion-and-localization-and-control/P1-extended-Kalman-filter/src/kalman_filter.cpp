#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  const MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
void KalmanFilter::UpdateCommon(const VectorXd& y)
{
    const MatrixXd PHt = P_ * H_.transpose();
    const MatrixXd S = H_ * PHt + R_;
    const MatrixXd K = PHt * S.inverse();
    
    x_ += K * y;
    P_ -= K * H_ * P_;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateCommon(y);
}
void KalmanFilter::NormalizeAngle(double& phi)
{
    phi = atan2(sin(phi), cos(phi));
}
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho_ = sqrt(px*px+py*py);
  const double eps = 1e-5;
  double phi_ = atan2(py, px);
  double rho_dot_ = (px*vx+py*vy)/std::max(eps, rho_);

  VectorXd z_pred(3);
  z_pred << rho_, phi_, rho_dot_;
  VectorXd y = z - z_pred;
  //normalize phi value in y
  NormalizeAngle(y(1));

  UpdateCommon(y);
}
