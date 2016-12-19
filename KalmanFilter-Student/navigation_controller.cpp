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
  ##Compute feedback for global navigation using Kfb (the state feedback gain in local cart frame)##

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

