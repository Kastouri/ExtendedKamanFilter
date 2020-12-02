#include "kalman_filter.h"
#include <iostream>

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

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
