#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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
  ekf_.P_ = MatrixXd(4, 4);
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

  ekf_.P_ << 1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1000, 0,
  0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
  0, 1, 0, 0;

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
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double meas_rho     = measurement_pack.raw_measurements_[0];
      double meas_phi     = measurement_pack.raw_measurements_[1];
      double meas_rho_dot = measurement_pack.raw_measurements_[2];

      double meas_px      = meas_rho* cos(meas_phi);
      double meas_py      = meas_rho* sin(meas_phi);
      double meas_vx      = meas_rho_dot* cos(meas_phi);
      double meas_vy      = meas_rho_dot* sin(meas_phi);
	  if ( meas_px < 0.0001 ) {
        meas_px = 0.0001;
      }
      if ( meas_py < 0.0001 ) {
        meas_py = 0.0001;
      }
      ekf_.x_ << meas_px , meas_py, meas_vx, meas_vy;
      }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "EKF : First measurement LASER" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_ ;
    is_initialized_ = true;
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
	
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  // 1. Modify the F matrix so that the time is integrated
  // 2. Set the process covariance matrix Q
  // 3. Call the Kalman Filter predict() function
  // 4. Call the Kalman Filter update() function
  //      with the most recent raw measurements_
  ekf_.F_ = MatrixXd::Identity(4, 4);
  
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  
  MatrixXd G =  MatrixXd(4, 2);
  G << (dt*dt)/2 , 0,
        0 , (dt*dt)/2,
        dt, 0,
        0, dt;
  MatrixXd G_t =  G.transpose();
  
  MatrixXd Q_v =  MatrixXd(2, 2);
  Q_v << noise_ax, 0,
            0, noise_ay;
  
  ekf_.Q_ = G*Q_v*G_t;
  
  
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
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
	ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
