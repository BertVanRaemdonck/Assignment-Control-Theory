#include "robot.h"
#include <math.h>
#include <stdio.h>

#define ENC1_PINA		18
#define ENC1_PINB		32
#define ENC2_PINA		19
#define ENC2_PINB		42
#define DIST1_PIN		A0
#define DIST2_PIN		A1
#define MOT1_PIN_IN1	8u
#define MOT1_PIN_IN2	12u
#define MOT1_PIN_EN		6u
#define MOT2_PIN_IN1	7u
#define MOT2_PIN_IN2	4u
#define MOT2_PIN_EN		5u
#define PENDULUM_PIN	A5
#define LED1_PIN 		34
#define LED2_PIN 		40
#define BATTERY_VOLTAGE	6000

#define RAD_TO_ENC ((34.0*11.0*2.0) / (2.0*3.141593))

Robot::Robot(uint8_t ID):
  _ID(ID),
  _type(20),
  _encoder1(new EncoderSensor(ENC1_PINA, ENC1_PINB)),
  _encoder2(new EncoderSensor(ENC2_PINA, ENC2_PINB)),
  _speed1(new DifferenceSensor(_encoder1, 10)), //,0.2f
  _speed2(new DifferenceSensor(_encoder2, 11)), //,0.2f
  _distance1(new Sharp41S(DIST1_PIN)),
  _distance2(new Sharp41S(DIST2_PIN)),
  _motor1(new L293D(MOT1_PIN_IN1, MOT1_PIN_IN2, MOT1_PIN_EN, BATTERY_VOLTAGE)),
  _motor2(new L293D(MOT2_PIN_IN1, MOT2_PIN_IN2, MOT2_PIN_EN, BATTERY_VOLTAGE)),
  _ekf(0.01)
{
  
}

void Robot::init() {
  //initialize the robot - sort of starting procedure
  resetKalmanFilter();
  resetEncoders();
}

double _error_integrator = 0;

void Robot::controllerHook() {
  //do something that is periodic
  //update speed sensors
  _speed1->readCalibratedValue();
  _speed2->readCalibratedValue();

  //Kalman filtering
  if (KalmanFilterEnabled()) {
    //prediction step
    ##implement prediction step using _ekf.PredictionStep(...)##
    //correction step
    //unless measurement is invalid
    if (System.getGPinInt(0)) {
      ##implement correction step using _ekf.CorrectionStep(...)##
    }
    
    //navigator
    if (navigationEnabled()) {
      //compute feedback
      float xref_arr [3][1] = { {System.getGPinFloat(0)}, {System.getGPinFloat(1)}, {System.getGPinFloat(2)} };
      float uff_arr [2][1] = { {System.getGPinFloat(3)}, {System.getGPinFloat(4)} };
      NavigationController::u_t unav = _nav.Controller(_ekf.getState(), NavigationController::x_t(xref_arr), NavigationController::u_t(uff_arr));
      //send wheel speed command
      NavigationController::u_t vLR = _nav.ControlToWheelSpeeds(unav);
      velocityControlUpdate(vLR(0), vLR(1));
    } else {
      _motor1->setBridgeVoltage(0);
      _motor2->setBridgeVoltage(0);
    }
  } else {
    _motor1->setBridgeVoltage(0);
    _motor2->setBridgeVoltage(0);
  }

  // int outputs
  System.setGPoutInt(0, _motor1->getBridgeVoltage());
  System.setGPoutInt(1, _motor2->getBridgeVoltage());
  System.setGPoutInt(2, System.getGPinInt(0));

  // float outputs
  System.setGPoutFloat(0, -wheelSpeedA());
  System.setGPoutFloat(1, wheelSpeedB());
  System.setGPoutFloat(2, _ekf.getStateStandardDeviation(0));
  System.setGPoutFloat(3, _ekf.getStateStandardDeviation(1));
  System.setGPoutFloat(4, _ekf.getStateStandardDeviation(2));
  System.setGPoutFloat(5, _ekf.getState(0));
  System.setGPoutFloat(6, _ekf.getState(1));
  System.setGPoutFloat(7, _ekf.getState(2));
}

void Robot::resetEncoders()
{
  _encoder1->init();
  _encoder2->init();
}

double Robot::wheelSpeedA()
{
  return _speed1->peekRawValue() * 0.5 * 0.0325 / RAD_TO_ENC;
}

double Robot::wheelSpeedB()
{
  return _speed2->peekRawValue() * 0.5 * 0.0325 / RAD_TO_ENC;
}

void Robot::resetKalmanFilter()
{
  const float Q[3][3] {  {##Q00##,       0,       0},
                         {      0, ##Q11##,       0},
                         {      0,       0, ##Q22##}
                      };
  const float R[2][2] {  {##R00##,       0},
                         {      0, ##R11##}
                      };
  const float P0[3][3] {  {##P00##,       0,       0},
                          {      0, ##P11##,       0},
                          {      0,       0, ##P22##}
                       };
  const float x0[3][1] {  {    ##X0##},
                          {    ##Y0##},
                          {##THETA0##}
                       };
  _ekf.setQ(EkfCart::Q_t(Q)*_ekf.getTs());
  _ekf.setR(EkfCart::R_t(R));
  _ekf.setState(EkfCart::x_t(x0));
  _ekf.setStateCovariance(EkfCart::P_t(P0));
  _ekf.setWallOne(1, 0, _distance1->readCalibratedValue() + ##alpha##);
  _ekf.setWallTwo(0, 1, _distance2->readCalibratedValue() + ##gamma##);
  _ekf.setCartParameters(##alpha##, ##beta##, ##gamma##);
}

void Robot::resetNavigationController()
{
  _nav.setCartParameters(##half wheel base##);
  const float Kfb[2][3] { {##kx##,      0,          0},
                          {     0, ##ky##, ##ktheta##}
                        };
  _nav.setFeedbackGainMatrix(NavigationController::K_t(Kfb));
}

double Robot::wrap2pi(double angle)
{
  while (angle < -M_PI) {
    angle += (2.0 * M_PI);
  }
  while (angle > M_PI) {
    angle -= (2.0 * M_PI);
  }
  return angle;
}

void Robot::setID(uint8_t ID) {
  _ID = ID;
}

uint8_t Robot::id()
{
  return _ID;
}

uint8_t Robot::type()
{
  return _type;
}

bool Robot::toggleButton(uint8_t button)
{
  _button_states[button] = !_button_states[button];
  return _button_states[button];
}

bool Robot::KalmanFilterEnabled()
{
  return _button_states[0];
}

bool Robot::navigationEnabled()
{
  return _button_states[1];
}

void Robot::button1callback()
{
  toggleButton(0);

  resetEncoders();

  System.println("Reset.");
}

void Robot::button2callback()
{
  resetKalmanFilter();
  if (toggleButton(1)) {
    System.println("Kalman filter enabled.");
  } else {
    System.println("Kalman filter disabled.");
  }
}
void Robot::button3callback()
{
  resetNavigationController();
  resetVelocityControl();
  if (toggleButton(2)) {
    System.println("Navigation enabled.");
  } else {
    System.println("Navigation disabled.");
  }
}

void Robot::button4callback()
{
  toggleButton(3);
}

void Robot::button5callback()
{
  toggleButton(4);
}

void Robot::button6callback()
{
  toggleButton(5);
}

void Robot::button7callback()
{
  toggleButton(6);
}

void Robot::button8callback()
{
  toggleButton(7);
}

#define MAXIMUM_CART_VELOCITY 0.4
void Robot::velocityControlUpdate(double setpoint_left, double setpoint_right)
{
  //setpoint governer
  if (setpoint_left > MAXIMUM_CART_VELOCITY) {
    setpoint_left = MAXIMUM_CART_VELOCITY;
  } else if (setpoint_left < -MAXIMUM_CART_VELOCITY) {
    setpoint_left = -MAXIMUM_CART_VELOCITY;
  }
  if (setpoint_right > MAXIMUM_CART_VELOCITY) {
    setpoint_right = MAXIMUM_CART_VELOCITY;
  } else if (setpoint_right < -MAXIMUM_CART_VELOCITY) {
    setpoint_right = -MAXIMUM_CART_VELOCITY;
  }

  int16_t motor1_voltage = ##Call your speed controller for left wheel which you already designed##
  int16_t motor2_voltage = ##Call your speed controller for right wheel which you already designed##

  _motor1->setBridgeVoltage(motor1_voltage);
  _motor2->setBridgeVoltage(motor2_voltage);
}
