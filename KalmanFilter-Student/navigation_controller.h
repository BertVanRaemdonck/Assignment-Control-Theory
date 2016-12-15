#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include <BasicLinearAlgebra.h>

class NavigationController {
  public:
    typedef Matrix<2, 3> K_t;
    typedef Matrix<3, 1> x_t;
    typedef Matrix<2, 1> u_t;

    void setFeedbackGainMatrix(const Matrix<2, 3>& Kfb_);
    void setCartParameters(const float a_);
    Matrix<2, 1> Controller(const Matrix<3, 1>& x, const Matrix<3, 1>& xref, const Matrix<2, 1>& uff) const; //compute control action utilde
    Matrix<2, 1> ControlToWheelSpeeds(const Matrix<2, 1>& input) const; //[v;w] -> [v_left;v_right]
    Matrix<2, 1> WheelSpeedsToControl(const Matrix<2, 1>& input) const; //[v_left;v_right] -> [v;w]

  private:
    Matrix<2, 3> Kfb; //feedback matrix
    Matrix<2, 2> T1; //transformation matrix [v; w] -> [v_left; v_right]
    Matrix<2, 2> T2; //transformation matrix [v_left; v_right] -> [v; w]
    float a; //half of wheel base (distance between wheels)
};

#endif /* NAVIGATION_CONTROLLER_H */
