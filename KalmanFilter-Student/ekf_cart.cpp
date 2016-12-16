#include "ekf_cart.h"
#include <math.h>
#include <microOS.h>

EkfCart::EkfCart(float Ts_) : KalmanFilter<3, 2, 2>(Ts_) {

}

EkfCart::EkfCart(float Ts_, const Q_t& Q_, const R_t& R_) : KalmanFilter<3, 2, 2>(Ts_, Q_, R_) {

}

void EkfCart::setWallOne(float a, float b, float c) {
  weq1(0) = a;
  weq1(1) = b;
  weq1(2) = c;
}

void EkfCart::setWallTwo(float a, float b, float c) {
  weq2(0) = a;
  weq2(1) = b;
  weq2(2) = c;
}

void EkfCart::setCartParameters(float alpha_, float beta_, float gamma_) {
  alpha = alpha_;
  beta = beta_;
  gamma = gamma_;
}

EkfCart::y_t EkfCart::hfun(const x_t& x) const {
  //implement measurement model: own code
  Matrix<2,1> y;
  
  y(0) = ( weq1(0)*(x(0)+alpha) + weq1(1)*x(1)         - weq1(2) ) / sqrt( pow(weq1(0),2) + pow(weq1(1),2) ) ;
  y(1) = ( weq2(0)*(x(0)-beta)  + weq2(1)*(x(1)+gamma) - weq2(2) ) / sqrt( pow(weq2(0),2) + pow(weq2(1),2) ) ;

  return y;
}

EkfCart::C_t EkfCart::Cfun(const x_t& x) const {
  //implement Jacobian of measurement function: own code
  Matrix<2,3> Jh;
  Jh.Fill(0.0);

  Jh(0,0) = weq1(0) / sqrt( pow(weq1(0),2) + pow(weq1(1),2) ) ;
  Jh(0,1) = weq1(1) / sqrt( pow(weq1(0),2) + pow(weq1(1),2) ) ;

  Jh(1,0) = weq2(0) / sqrt( pow(weq2(0),2) + pow(weq2(1),2) ) ;
  Jh(1,1) = weq2(1) / sqrt( pow(weq2(0),2) + pow(weq2(1),2) ) ;

  return Jh;
}

EkfCart::x_t EkfCart::ffun(const x_t& x, const u_t& u) const {
  //implement discrete-time state equation: own code
  Matrix<3,1> x_next;

  x_next(0) = x(0) + Ts*cos(x(2))*u(0) ;
  x_next(1) = x(1) + Ts*sin(x(2))*u(0) ;
  x_next(2) = x(2) + Ts*u(1) ;

  return x_next;  
}

EkfCart::A_t EkfCart::Afun(const x_t& x, const u_t& u) const {
  //implement Jacobian of discrete-time state equation
  Matrix<3,3> Jf;
  Jf.Fill(0.0);

  Jf(0,0) = 1.0 ;
  Jf(1,1) = 1.0 ;
  Jf(2,2) = 1.0 ;

  Jf(0,2) = -Ts*u(0)*sin(x(2)) ;
  Jf(1,2) =  Ts*u(0)*sin(x(2)) ;

  return Jf; 
}

