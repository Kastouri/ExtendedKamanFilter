#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  //initializing the laser measurement matrix
  H_laser_ << 1., 0., 0., 0.,
              0., 1., 0., 0.;
  
  //initializing the radars covariance matrix 
  Hj_ << 0., 0., 0., 0.,
         0., 0., 0., 0.,
         0., 0., 0., 0.; 
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    //creating the covariance matrix 
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 10., 0., 0., 0.,
                0., 10., 0., 0.,
                0., 0., 1000., 0.,
                0., 0., 0., 1000.;
    // TODO: Change this and use ekf.init()
    // initialize state matrix
    ekf_.F_ = MatrixXd(4,4);
    // initialize the process noise convariance matrix
    ekf_.Q_ = MatrixXd(4,4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      ekf_.x_ << rho * cos(phi), // initialize px with rho * cos phi
                 rho * sin(phi), // initialize py with rho * sin phi
                 0.0, // initialize vx with 0
                  0.0; // initialize vy with 0
      cout << "first measurement was a RADAR measurement !!!!!!!!!!!!!!!!!" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1],
                0.0,
                0.0;

      cout << "first measurement was a LASER measurement !!!!!!!!!!!!!!!!!" << endl;
    }
    previous_timestamp_ = measurement_pack.timestamp_ ;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "DEBUGGING fusionEKF.cpp LINE 100: Initialization DONE!" << endl;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // calculating the elapsed time
  double dt; // elapsed time
  double noise_ax = 9.;
  double noise_ay = 9.;

  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_ ;

  //cout << "DEBUGGING fusionEKF.cpp LINE 125: elapsed time calculated to dt ="<< dt << endl;
  //cout << "DEBUGGING fusionEKF.cpp LINE 121: Starting F Update"<< endl;
  // updating the state transition matrix F
  ekf_.F_ <<  1., 0., dt, 0.,
              0., 1., 0., dt,
              0., 0., 1., 0,
              0., 0., 0.,  1.;

  //cout << "DEBUGGING fusionEKF.cpp LINE 128: State Matrix Updated!" << endl;
  // updating the process noise covariance matrix
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double c1 = dt_4 / 4;
  double c2 = dt_3 / 2;
  double c3 = dt_2;

  ekf_.Q_ << c1 * noise_ax, 0,             c2 * noise_ax, 0,
            0,              c1 * noise_ay, 0,             c2 * noise_ay,
            c2 * noise_ax,  0,             c3 * noise_ax, 0,
            0,              c2 * noise_ay, 0,             c3 * noise_ay;

  //cout << "DEBUGGING fusionEKF.cpp LINE 142: Process Cov. Matrix Updated!" << endl;
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // TODO: don't calculate jacobian if division by zero will happen, don't update
    double px = measurement_pack.raw_measurements_[0];
    double py =  measurement_pack.raw_measurements_[1];
    double c1 = px*px+py*py;
    if (fabs(c1) < 0.0001) {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return;
    }
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    cout << "Jacobian Calculated: " << endl;
    cout << Hj_ << endl;
    ekf_.H_ = MatrixXd(3,4);
    ekf_.H_ = Hj_;
    ekf_.R_  = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.H_ = MatrixXd(2,4);
    ekf_.H_ = H_laser_;
    ekf_.R_  = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }
  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
