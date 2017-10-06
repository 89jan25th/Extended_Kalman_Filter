#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px, py, vx, vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];
  
  float rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);  // returns values between -pi and pi
  
  // if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
  if(rho < 0.000001)
    rho = 0.000001;
  
  rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  
  VectorXd y = z - z_pred;

  // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= 2*M_PI;
  }

  while(y(1) < -M_PI){
    y(1) += 2*M_PI;
  }

  // following is exact the same as in the function of KalmanFilter::Update()
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
