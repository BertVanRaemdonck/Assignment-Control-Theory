#ifndef EKF_CART_H
#define EKF_CART_H

#include "kalman_filter.h"

class EkfCart : public KalmanFilter<3, 2, 2> {
  public:
    typedef KalmanFilter<3, 2, 2> KF;
    typedef KF::xvec_t x_t;
    typedef KF::uvec_t u_t;
    typedef KF::yvec_t y_t;
    typedef KF::xxmat_t A_t;
    typedef KF::xxmat_t Q_t;
    typedef KF::xxmat_t P_t;
    typedef KF::yymat_t R_t;
    typedef KF::yxmat_t C_t;

    EkfCart(float Ts_);
    EkfCart(float Ts_, const Q_t& Q_, const R_t& R_);
    void setWallOne(float a, float b, float c);
    void setWallTwo(float a, float b, float c);
    void setCartParameters(float alpha_, float beta_, float gamma_);

  private:
    Matrix<3, 1> weq1; //Wall equation: a1*X+b1*Y-c1==0, a1:=weq1(0), b1:=weq1(1), c1:=weq1(2)
    Matrix<3, 1> weq2; //Wall equation: a2*X+b2*Y-c2==0, a2:=weq2(0), b2:=weq2(1), c2:=weq2(2)
    float alpha; //distance front IR-sensor to wheel axis
    float beta; //distance side IR-sensor to wheel axis
    float gamma; //distance side IR-sensor to center symmetry axis

  protected:
    y_t hfun(const x_t& x) const; //measurement equation
    C_t Cfun(const x_t& x) const; //Jacobian of measurement equation
    x_t ffun(const x_t& x, const u_t& u) const; //state-update equation
    A_t Afun(const x_t& x, const u_t& u) const; //Jacobian of state-update equation
};

#endif /* EKF_CART_H */
