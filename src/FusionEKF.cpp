#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
        0, 0.0225;
        //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  H_laser_ = MatrixXd(2, 4);
  // quote: "Another reason could be improper initialization of matrices.
  // Doing MatrixXd(3, 3) will fill the matrix with random values.
  // Always remember to initialize to zero by doing MatrixXd::Zero(3, 3) or matrix.fill(0.0)."
  // in my case, I forgot to assign values to H_laser :(
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0 ;

  // cout << "H_laser_: " << endl << H_laser_ << endl;
  Hj_ = MatrixXd(3, 4);

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // But I can't set noises here, it should be set in ProcessMeasurement() as local var.
  cout << "FusionEKF constructor " << endl;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  float noise_ax = 9 ;
  float noise_ay = 9 ;
  // todo, long long or float?
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  // cout << "dt :" << dt << endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    // why is it 1, 1, 1, 1 ???

    // cout << "measurement_pack :" << measurement_pack.raw_measurements_ << endl ;
    ekf_.F_ = MatrixXd(4, 4) ;
    ekf_.F_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1 ;
    // //set the process covariance matrix Q
    // ekf_.Q_ = MatrixXd(4, 4);
    // ekf_.Q_ <<  0, 0, 0, 0,
    //             0, 0, 0, 0,
    //             0, 0, 0, 0,
    //             0, 0, 0, 0 ;
    // todo , why P_ init as 1000 ?
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    // ekf_.init();

    // CRITERIA: Your Kalman Filter algorithm handles the first measurements appropriately.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      cout << "RADAR : measurement_pack.raw_measurements_ : " << measurement_pack.raw_measurements_;
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      double px = cos(phi) * rho;
      double py = sin(phi) * rho;
      double vx = cos(phi) * rho_dot;
      double vy = sin(phi) * rho_dot;
      ekf_.x_ <<  px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      cout << "LASER measurement_pack :" << endl << measurement_pack.raw_measurements_ << endl;
      double px = measurement_pack.raw_measurements_(0);
      double py = measurement_pack.raw_measurements_(1);
      double vx = 0.0;
      double vy = 0.0;
      ekf_.x_ <<  px, py, vx, vy;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //  cout << " ekf_.F_ :" << ekf_.F_ << endl ;
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;
  //  cout << " dt :" << dt << endl ;
   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  // CRITERIA: Your Kalman Filter algorithm first predicts then updates.
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // CRITERIA: Your Kalman Filter can handle radar and lidar measurements.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_) ;
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_) ;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
