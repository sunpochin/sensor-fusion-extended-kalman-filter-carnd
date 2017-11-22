#include <iostream>
// #define _USE_MATH_DEFINES
// #include <math.h>
// #define M_PI (3.14159265358979323846)

using namespace std;
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
    * predict the state
  */
  cout << "KalmanFilter::Predict()" << endl ;

  // cout << "x_ = F_ * x_;" << endl ;
  // cout << "x_ :" << x_ << endl ;
  // cout << "F_ :" << F_ << endl ;
  // cout << "P_ :" << P_ << endl ;
  // cout << "Q_ :" << Q_ << endl ;
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
  // cout << "P_ :" << P_ << endl ;
}

bool debug = false;
// debug = true;
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "KalmanFilter::Update()" << endl ;

  VectorXd z_pred = H_ * x_;
  if (debug) {
    cout << "z_pred :" << z_pred << endl;
    cout << "z :" << z << endl;
  }
	VectorXd y = z - z_pred;
  if (debug) {
    cout << "y :" << y  << endl;
  }
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
  if (debug) {
    cout << "S :" << S  << endl;
  }
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
  if (debug) {
    cout << "PHt :" << PHt  << endl;
  }
	MatrixXd K = PHt * Si;
  if (debug) {
    cout << "K :" << K  << endl;
  }

	 //new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  cout << "KalmanFilter::UpdateEKF()" << endl ;

  // Predicted location in polar coordinates.
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  // from lecture: "Tips and Tricks"
  // Normalizing Angles
  // In C++, atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements,
  // the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi.
  // The Kalman filter is expecting small angle values between the range -pi and pi.
  // HINT: when working in radians, you can add 2π or subtract 2π until the angle is within the desired range.

  // Avoid Divide by Zero throughout the Implementation
  // Before and while calculating the Jacobian matrix Hj, make sure your code avoids dividing by zero.
  // For example, both the x and y values might be zero or px*px + py*py might be close to zero.
  // What should be done in those cases?
  // rho: ρ , phi: ϕ,
  // range: rho, radial distance from origin.
  float rho = sqrtf(powf(px, 2) + powf(py, 2));
  // bearing: phi, angle between rho and x.
  float phi = atan2(py, px);
//  float phi = atan2f(vy, vx);
  // radial velocity: change of rho (range rate)
  // prevent divide by zero
  float eps = 0.000001;
  // if (fabs(px) < eps ) {
  //   cout << "Error x_ to polar coordinates: Division by Zero" << endl;
  //   px = eps;
  // }
  // if (fabs(py) < eps ) {
  //   cout << "Error x_ to polar coordinates: Division by Zero" << endl;
  //   py = eps;
  // }
  if (fabs(rho) < eps ) {
      rho = eps;
  }
  float rho_dot = (px * vx + py * vy) / rho;

  // measurement function h(x')
  VectorXd hofx(3);
  hofx << rho, phi, rho_dot;

  // Intermediate calculations.
  MatrixXd Ht = H_.transpose();
  // MatrixXd Ht = hx.transpose();
  VectorXd y = z - hofx;
  float PI = 3.14159265358979323846;
  // if (y(1) > PI) {
  //   y(1) = y(1) - 2 * PI;
  // }
  // if (y(1) < -1 * PI) {
  //   y(1) = y(1) + 2 * PI;
  // }
  // todo: what's the difference???
  while (y(1)>PI) {
    y(1) -= 2 * PI;
  }
  while (y(1)<-PI) {
    y(1) += 2 * PI;
  }

  // cout << "P_ :" << P_ << endl;
  // cout << "R_ :" << R_ << endl;
  // cout << "H_ * P_ * Ht :" << H_ * P_ * Ht << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  long size = x_.size();
  MatrixXd I = MatrixXd::Identity(size, size);
  // cout << "rho :" << rho  << endl;
  // cout << "phi : " << phi << endl;
  // cout << "H_ :" << H_ << endl;
  // cout << "hofx :" << hofx << endl;
  // cout << "y :" << y << endl;
  // cout << "S :" << S << endl;

  // Update state and covariance mats.
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
