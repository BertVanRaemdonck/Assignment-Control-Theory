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
  ##implement measurement model##
}

EkfCart::C_t EkfCart::Cfun(const x_t& x) const {
  ##implement Jacobian of measurement function##
}

EkfCart::x_t EkfCart::ffun(const x_t& x, const u_t& u) const {
  ##implement discrete-time state equation##
}

EkfCart::A_t EkfCart::Afun(const x_t& x, const u_t& u) const {
  ##implement Jacobian of discrete-time state equation##
}

