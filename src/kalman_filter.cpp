#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {first=true;}

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

  //calulcate H.tranpose only once
  if(first)
    Htranspose_ = H_.transpose();
  
  first=false;

  MatrixXd P1 = P_ * Htranspose_;
  MatrixXd S = H_ * P_ * Htranspose_ + R_;
  MatrixXd KG = P1 * S.inverse();
  
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
  // The difference from Update is we need to change H_ * x_ for radar . we need to convert the prediction which is in x,y (cartesian) domain to measurement which is in rho, theta, ro (polar) domain. the reverse of when getting meas from radar
  VectorXd h_of_x(3);
  float x = x_[0];
  float y = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float p = sqrt(x*x + y*y); // rho
  float q = atan2(y,x); //theta


  float pb;
  if(p<0.001)// divide by 0
    pb = (x*vx + y * vy)/last_p;
  else{
    pb = (x*vx + y * vy)/p; // rho_dot
    last_p=p;
  }

  h_of_x << p,q,pb;

  
  MatrixXd P1 = P_ * H_.transpose();
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd KG = P1 * S.inverse();
  
  // new state
  // then calculate new estimate
  // EST(new) = EST(old)  + KG(meas - EST(old)
  
  MatrixXd KG_term = z - ( h_of_x); //  this is meas - EST(OLD)/z_pred; H for matrix conversion

  //normalizing theta of KG_term
  while(KG_term(1)>M_PI)
    KG_term(1)-= 2*M_PI;
  while(KG_term(1)<-M_PI)
    KG_term(1)+= 2*M_PI;


  x_ = x_ + (KG * KG_term);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - KG*H_)  * P_; // Error(new) = (1-KG) Error(old). I is used instead of 1 for matrix

  
}
