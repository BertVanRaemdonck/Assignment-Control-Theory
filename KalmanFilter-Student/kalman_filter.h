#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <BasicLinearAlgebra.h>
#include <math.h>

template<int nx, int nu, int ny>
class KalmanFilter
{
  protected:
    typedef Matrix<nx, 1> xvec_t;
    typedef Matrix<nu, 1> uvec_t;
    typedef Matrix<ny, 1> yvec_t;
    typedef Matrix<nx, nx> xxmat_t;
    typedef Matrix<ny, ny> yymat_t;
    typedef Matrix<nx, ny> xymat_t;
    typedef Matrix<ny, nx> yxmat_t;

    float Ts;
    Matrix<nx, nx> Q;
    Matrix<ny, ny> R;

    Matrix<nx, 1> xhat;
    Matrix<nx, nx> Phat;

    virtual Matrix<ny, 1> hfun(const Matrix<nx, 1>& x) const {};
    virtual Matrix<ny, nx> Cfun(const Matrix<nx, 1>& x) const {};
    virtual Matrix<nx, 1> ffun(const Matrix<nx, 1>& x, const Matrix<nu, 1>& u) const {};
    virtual Matrix<nx, nx> Afun(const Matrix<nx, 1>& x, const Matrix<nu, 1>& u) const {};

  public:
    KalmanFilter(float Ts_);
    KalmanFilter(float Ts_, const Matrix<nx, nx>& Q_, const Matrix<ny, ny>& R_);
    void reset();

    void PredictionStep(const Matrix<nu, 1>& u);
    void CorrectionStep(const Matrix<ny, 1>& y);

    float getTs() const;
    const Matrix<nx, nx>& getQ() const;
    const Matrix<ny, ny>& getR() const;
    const Matrix<nx, 1>& getState() const;
    float getState(int idx) const;
    const Matrix<nx, nx>& getStateCovariance() const;
    float getStateStandardDeviation(int idx) const;

    void setTs(float Ts_);
    void setQ(const Matrix<nx, nx>& Q_);
    void setR(const Matrix<ny, ny>& R_);
    void setState(const Matrix<nx, 1>& xhat_);
    void setStateCovariance(const Matrix<nx, nx>& Phat_);

    int get_nx() const {
      return nx;
    }
    int get_nu() const {
      return nu;
    }
    int get_ny() const {
      return ny;
    }

};

template<int nx, int nu, int ny>
KalmanFilter<nx, nu, ny>::KalmanFilter(float Ts_) {
  Ts = Ts_;
  for (int i = 0; i < nx; ++i) for (int ii = 0; ii < nx; ++ii) Q(i, ii) = i == ii ? 1 : 0;
  for (int i = 0; i < ny; ++i) for (int ii = 0; ii < ny; ++ii) R(i, ii) = i == ii ? 1 : 0;
}

template<int nx, int nu, int ny>
KalmanFilter<nx, nu, ny>::KalmanFilter(float Ts_, const Matrix<nx, nx>& Q_, const Matrix<ny, ny>& R_) {
  Ts = Ts_;
  Q = Q_;
  R = R_;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::reset() {

}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::PredictionStep(const Matrix<nu, 1>& u) {
  Matrix<nx, nx> A = Afun(xhat, u);
  xhat = ffun(xhat, u);
  Phat = A * Phat * A.Transpose() + Q;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::CorrectionStep(const Matrix<ny, 1>& y) {
  Matrix<ny, nx> C = Cfun(xhat);
  Matrix<ny, ny> S = R + C * Phat * C.Transpose();
  Matrix<nx, ny> L = Phat * C.Transpose() * S.Inverse();
  Matrix<ny, 1> err = y - hfun(xhat);

  Matrix<nx, nx> eye;
  eye(0, 0) = 1; eye(0, 1) = 0; eye(0, 2) = 0;
  eye(1, 0) = 0; eye(1, 1) = 1; eye(1, 2) = 0;
  eye(2, 0) = 0; eye(2, 1) = 0; eye(2, 2) = 1;

  Matrix<nx, nx> IKH = eye - L * C;
  xhat += L * err;
  Phat = IKH * Phat * IKH.Transpose() - L * R * L.Transpose(); //equivalent to (I-K*H)*Phat;
}

template<int nx, int nu, int ny>
float KalmanFilter<nx, nu, ny>::getTs() const {
  return Ts;
}

template<int nx, int nu, int ny>
const Matrix<nx, nx>& KalmanFilter<nx, nu, ny>::getQ() const {
  return Q;
}

template<int nx, int nu, int ny>
const Matrix<ny, ny>& KalmanFilter<nx, nu, ny>::getR() const {
  return R;
}

template<int nx, int nu, int ny>
const Matrix<nx, 1>& KalmanFilter<nx, nu, ny>::getState() const {
  return xhat;
}

template<int nx, int nu, int ny>
float KalmanFilter<nx, nu, ny>::getState(int idx) const {
  return xhat(idx, 0);
}

template<int nx, int nu, int ny>
const Matrix<nx, nx>& KalmanFilter<nx, nu, ny>::getStateCovariance() const {
  return Phat;
}

template<int nx, int nu, int ny>
float KalmanFilter<nx, nu, ny>::getStateStandardDeviation(int idx) const {
  if ((idx < 0) || (idx >= nx)) return 0;
  float var = Phat(idx, idx);
  return sqrt(var);
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::setTs(float Ts_) {
  Ts = Ts_;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::setQ(const Matrix<nx, nx>& Q_) {
  Q = Q_;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::setR(const Matrix<ny, ny>& R_) {
  R = R_;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::setState(const Matrix<nx, 1>& xhat_) {
  xhat = xhat_;
}

template<int nx, int nu, int ny>
void KalmanFilter<nx, nu, ny>::setStateCovariance(const Matrix<nx, nx>& Phat_) {
  Phat = Phat_;
}

#endif /* KALMAN_FILTER_H */
