#include "kalman_filter.h"
#include <iostream>
#include <math.h>

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
    
    // Laser updates
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    
    UpdateCommon(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
//  update the state by using Extended Kalman Filter equations
    
    double px, py, vx, vy, rho, phi, rhoDot;
    
    px = x_[0];
    py = x_[1];
    vx = x_[2];
    vy = x_[3];
    
    if (fabs(px) < 0.00001) {
        px = 0.0001;
    }
    if (fabs(py) < 0.00001) {
        py = 0.0001;
    }
    
    rho = sqrt(px*px + py*py);
    if (fabs(rho) < 0.0001) {
        rho = 0.0001;
    }
    phi = atan2(py,px);
    rhoDot = (px*vx+py*vy)/rho;
    
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rhoDot;
    VectorXd y = z - z_pred;
    
    
//  Fix large angles by adding/ subracting a full rotation:
    while (y(1) > M_PI){
        y(1) -= 2*M_PI;
    }
    while (y(1) < -M_PI) {
        y(1) += 2*M_PI;
    }
    
    UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd &y){
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
