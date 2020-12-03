#include "kalman_filter.h"
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  /**
   * TODO: predict the state
   */
  //cout << "DEBUGGING LINE 32: tried prediction!" << endl;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  //cout << "DEBUGGING LINE 41: tried update!" << endl;
  //cout << "DEBUGGING z=" << z << " H= "<< H_;
  VectorXd y = z - H_ * x_;
  //cout << "DEBUGGING kalman_filter.cpp  1: setting up variables!" << endl;

  MatrixXd Ht = H_.transpose();
  //cout << "DEBUGGING kalman_filter.cpp  2: setting up variables!" << endl;

  MatrixXd S = H_ * P_ * Ht + R_;
  //cout << "DEBUGGING kalman_filter.cpp  3: setting up variables!" << endl;

  MatrixXd Si = S.inverse();
  //cout << "DEBUGGING kalman_filter.cpp  4: setting up variables!" << endl;

  MatrixXd K =  P_ * Ht * Si; 
  //cout << "DEBUGGING kalman_filter.cpp 5 : setting up variables!" << endl;
  
  MatrixXd I = MatrixXd::Identity(4, 4);
  //cout << "DEBUGGING kalman_filter.cpp LINE 48: setting up variables!" << endl;

  x_ = x_ + (K * y);
  ///cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  //cout << "DEBUGGING kalman_filter.cpp LINE 61: X updated successfully!" << endl;
  P_ = (I - K * H_) * P_;
  //cout << "updated using LIDAR" << endl;
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px= x_(0);
  double py= x_(1);
  double vx= x_(2);
  double vy= x_(3);
  //cout << "px="<< px << endl;
  //cout << "py="<< py << endl;
  //cout << "vx="<< vx << endl;
  //cout << "vy="<< vy << endl;

  double px_2 = px* px;
  double py_2 = py* py;
  double rho_pred = sqrt(px_2 + py_2);
  if (rho_pred < 0.0000001){
    rho_pred = 0.0000001;
  }
  //cout << "angle rate predicted to be rho="<< rho_pred << endl;
  
  double phi_pred = atan2(py, px);
  //cout << "angle predicted to be phi="<< phi_pred * 180 / 3.14 << endl;
  double rho_dot_pred = (px*vx + py*vy)/(sqrt(px_2 + py_2)); 
  //cout << "angle rate predicted to be rho_dot="<< rho_dot_pred << endl;
  
  VectorXd z_pred(3);
  z_pred << rho_pred, phi_pred, rho_dot_pred;
  VectorXd y = z - z_pred;
  //cout << "Error Vector y= "<< y <<" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;
  while ((y(1) > M_PI) ||  (y(1) < -M_PI)){
    if (y(1) >= M_PI){
      y(1) -= 2 * M_PI;
    } else if (y(1) < -M_PI)
    {
      y(1) += 2 * M_PI;
    }
  }
  MatrixXd Ht = H_.transpose();

  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd Si = S.inverse();

  MatrixXd K =  P_ * Ht * Si; 
  
  MatrixXd I = MatrixXd::Identity(4, 4);

  x_ = x_ + (K * y);

  P_ = (I - K * H_) * P_;
  
  //cout << "updated using RADAR" << endl;
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl; 
}
