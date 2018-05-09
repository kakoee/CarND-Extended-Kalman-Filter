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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // division by S. P is Error estimated, and R is the meas error.
  MatrixXd P1 = P_ * H_.transpose();
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd KG = P1 * S_.inverse();
  
  // new state
  // then calculate new estimate
  // EST(new) = EST(old)  + KG(meas - EST(old)
  MatrixXd KG_term = z - ( H_ * x_); //  this is meas - EST(OLD); H for matrix conversion
  x_ = x_ + (KG * KG_term);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - KG*H_)  * P_; // Error(new) = (1-KG) Error(old). I is used instead of 1 for matrix
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
