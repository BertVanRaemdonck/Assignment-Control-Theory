#include "navigation_controller.h"
#include <math.h>

#define fsign(a) ((a>0) - (a<0))

void NavigationController::setCartParameters(const float a_) {
  a = a_;

  //T1: transformation matrix [v; w] -> [v_left; v_right], own code
  T1(0,0) = 1;
  T1(0,1) = -a;
  T1(1,0) = 1;
  T1(1,1) = a;

  //T2: transformation matrix [v_left; v_right] -> [v; w], own code
  T2(0,0) = 1/2;
  T2(0,1) = 1/2;
  T2(1,0) = -1/(2*a);
  T2(1,1) = 1/(2*a);
}

void NavigationController::setFeedbackGainMatrix(const Matrix<2, 3>& Kfb_) {
  Kfb = Kfb_;
}

Matrix<2, 1> NavigationController::Controller(const Matrix<3, 1>& x, const Matrix<3, 1>& xref, const Matrix<2, 1>& uff) const {
  Matrix<2, 1> u;

  // Feedback term
  // still need to fill in
  //##Compute feedback for global navigation using Kfb (the state feedback gain in local cart frame)##

  // \begin{own code}
  // calculate global error of x and y
  //Matrix<3, 1> e_glob = xref - x;
  Matrix<2, 1> e_glob_xy;
  e_glob_xy(0) = xref(0) - x(0);
  e_glob_xy(1) = xref(1) - x(1);

  // calculate transformation matrix from global to local for the errors in x and y
  Matrix<2, 2> Rot;
  Rot(0,0) =  cos(x(2));
  Rot(0,1) = -sin(x(2));
  Rot(1,0) =  sin(x(2));
  Rot(1,1) =  cos(x(2));

  // calculate local error of x and y
  Matrix<2, 1> e_loc_xy = Rot * e_glob_xy;

  // calculate total local error
  Matrix<3, 1> e_loc;
  e_loc(0) = e_loc_xy(0);
  e_loc(1) = e_loc_xy(1);
  e_loc(2) = xref(2) - x(2);

  // calculate feedback
  u = Kfb * e_loc;
  
  // \end{own code}  

  // Add feed-forward term
  u += uff;

  return u;
}

Matrix<2, 1> NavigationController::ControlToWheelSpeeds(const Matrix<2, 1>& input) const {
  //Transform to [v_left, v_right]
  return T1 * input;
}

Matrix<2, 1> NavigationController::WheelSpeedsToControl(const Matrix<2, 1>& input) const {
  //Transform to [v, w]
  return T2 * input;
}

