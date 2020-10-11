#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  
  //TODO: predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; 
  
}

void KalmanFilter::Update(const VectorXd &z) {
  
  //TODO: update the state by using Kalman Filter equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // y = z - H*x' becomes y = z - h(x') for polar radar coordinates
  // use trigonometry to convert each x,y,vx,vy cartesian coordinate to polar coordinates
  float ppx = x_[0];
  float ppy = x_[1];
  float vpx = x_[2];
  float vpy = x_[3];
  //calculate rho
  float hxp0 = sqrt(ppx*ppx + ppy*ppy);
  //calculate phi
  float hxp1 = atan2(ppy,ppx);
  //calculate rho dot
  float hxp2;
  //prevent division by zero
  if (fabs(hxp0) < 0.0001) {
    hxp2 = 0;
  }
  else {
    hxp2 = ((ppx*vpx + ppy*vpy)/hxp0);
  }
  
    
  VectorXd z_pred(3);
  z_pred << hxp0, hxp1, hxp2;  
  VectorXd y = z - z_pred;
  
  
  //make sure phi is between -pi and pi
  float pi = 2*acos(0.0);
  while (y[1] >= pi) {
    y[1] -= 2*pi;
    }
  while (y[1] < -pi) {
    y[1] += 2*pi;
   }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;   
  
}
