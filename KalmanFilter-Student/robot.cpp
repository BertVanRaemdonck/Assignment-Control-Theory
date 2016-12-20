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

// defining own variables
float RADIUS_WHEEL = 0.0325; // [m] see function wheelSpeedA()

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
  // own code
  resetVelocityControl();
}

double _error_integrator = 0;

void Robot::controllerHook() {
  // \begin{own code}
  int enc1_curr = _encoder1 ->readRawValue();
  int enc2_curr = _encoder2 ->readRawValue();
  // \end{own code}
  
  
  //do something that is periodic
  //update speed sensors
  _speed1->readCalibratedValue();
  _speed2->readCalibratedValue();

  //Kalman filtering
  if (KalmanFilterEnabled()) { // press button 1 to toggle
    //prediction step : own code
    float u_ff_pred [2][1] = { {System.getGPinFloat(3)}, {System.getGPinFloat(4)} }; // contains the same as uff_arr, but separate variable just in case navigationEnabled is False
    
    _ekf.PredictionStep(NavigationController::u_t(u_ff_pred));
      
    //correction step
    //unless measurement is invalid
    if (System.getGPinInt(0)) { 
      // own code
      Matrix<2,1> y_meas;
      y_meas(0) = _distance1->readCalibratedValue();
      y_meas(1) = _distance2->readCalibratedValue();
      
      _ekf.CorrectionStep(y_meas);
    }
    
    //navigator
    if (navigationEnabled()) { // press button 2 to toggle
      //compute feedback
      float xref_arr [3][1] = { {System.getGPinFloat(0)}, {System.getGPinFloat(1)}, {System.getGPinFloat(2)} };
      float uff_arr [2][1] = { {System.getGPinFloat(3)}, {System.getGPinFloat(4)} };
      NavigationController::u_t unav = _nav.Controller(_ekf.getState(), NavigationController::x_t(xref_arr), NavigationController::u_t(uff_arr));
      //send wheel speed command
      NavigationController::u_t vLR = _nav.ControlToWheelSpeeds(unav);
      velocityControlUpdate(vLR(0), vLR(1));

  
    } else {
      // \begin{own code}
      // just drive the cart with the feedforward velocity     
      NavigationController::u_t vLR = _nav.ControlToWheelSpeeds(NavigationController::u_t(u_ff_pred));
      velocityControlUpdate(-vLR(0), vLR(1));    

      //System.setGPoutFloat(0, uff_arr(0));
      //System.setGPoutFloat(1, uff_arr(1));
      //System.setGPoutFloat(2, u_ff_pred[0][1]);
      
      // \end{own code}

      /*
      _motor1->setBridgeVoltage(0);
      _motor2->setBridgeVoltage(0);
      */
    }
  } else {
    resetVelocityControl(); // own code
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
  //System.setGPoutFloat(2, _ekf.getStateStandardDeviation(0));
  //System.setGPoutFloat(3, _ekf.getStateStandardDeviation(1));
  //System.setGPoutFloat(4, _ekf.getStateStandardDeviation(2));
  System.setGPoutFloat(5, _ekf.getState(0));
  System.setGPoutFloat(6, _ekf.getState(1));
  System.setGPoutFloat(7, _ekf.getState(2));

  //own code
  
  System.setGPoutFloat(2, System.getGPinFloat(0)); // x_ff
  System.setGPoutFloat(3, System.getGPinFloat(1)); // y_ff
  System.setGPoutFloat(4, System.getGPinFloat(2)); // theta _ff

  // \begin{own code}
  enc1_prev = enc1_curr;
  enc2_prev = enc2_curr;
  // \end{own code}
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
  // these values have to be tuned: own code
  const float Q[3][3] {  {1e-7   ,       0,       0},
                         {      0, 1e-7   ,       0},
                         {      0,       0, 1e-7   }
                      };
  const float R[2][2] {  {1e-4   ,       0},
                         {      0, 1e-4   }
                      };
  const float P0[3][3] {  {1e-3   ,       0,       0},
                          {      0, 1e-3   ,       0},
                          {      0,       0, 1e-3   }
                       };
  // x0 , y0 , theta0: own code
  const float x0[3][1] {  { 0 },
                          { 0 },
                          { 0 }
                       };
  _ekf.setQ(EkfCart::Q_t(Q)*_ekf.getTs());
  _ekf.setR(EkfCart::R_t(R));
  _ekf.setState(EkfCart::x_t(x0));
  _ekf.setStateCovariance(EkfCart::P_t(P0));
  _ekf.setWallOne(1, 0, _distance1->readCalibratedValue() + 0.0875); // alpha
  _ekf.setWallTwo(0, 1, _distance2->readCalibratedValue() + 0.0855); // gamma
  _ekf.setCartParameters(0.0875, 0.065, 0.0855); // alpha, beta, gamma
}

void Robot::resetNavigationController()
{
  // still need to fill in!
  _nav.setCartParameters(0.1695/2.0); // own code: measured value for a
//  const float Kfb[2][3] { {##kx##,      0,          0},
//                          {     0, ##ky##, ##ktheta##}
//                        };
//  _nav.setFeedbackGainMatrix(NavigationController::K_t(Kfb));
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
  return _button_states[1];
}

bool Robot::navigationEnabled()
{
  return _button_states[2];
}

void Robot::button1callback()
{
  toggleButton(0);

  resetEncoders();
  // \begin(own_code)
  resetNavigationController();
  resetKalmanFilter();
  resetVelocityControl();

  //_button_states[1] = 0;
  //_button_states[2] = 0;
  

  // \end(own_code)

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

  // \begin{own code}


  System.setGPoutFloat(0, setpoint_left);
  System.setGPoutFloat(1, setpoint_right);

  // convert speeds to [enc/s] because that's how our speed controller works
  double speed1_des_double = (setpoint_left * 2.0 * RAD_TO_ENC)/RADIUS_WHEEL;
  double speed2_des_double = (setpoint_right * 2.0 * RAD_TO_ENC)/RADIUS_WHEEL;
  // convert desired speeds to floats because that's how our speed controller works
  float speed1_des = (float) speed1_des_double;
  float speed2_des = (float) speed2_des_double;
  // call the speed controller with the converted speeds      
  controller_speed(speed1_des, speed2_des);
  
  // \end{own code}
  
  /* not needed because speed gets written to the motors within the controller function itself
   
  int16_t motor1_voltage = uk_speed1[0];  // ##Call your speed controller for left wheel which you already designed##
  int16_t motor2_voltage = uk_speed2[0]   // ##Call your speed controller for right wheel which you already designed##
  
  _motor1->setBridgeVoltage(motor1_voltage);
  _motor2->setBridgeVoltage(motor2_voltage);
  */
}



// own code for speed controller

void Robot::controller_speed(float speed1_des, float speed2_des)
{
   /**
   * This is the implemented velocity controller.
   * This can be used to control the velocity of the motors seperatly.
   */
   
  // read encoder values
  int enc1 = _encoder1 ->readRawValue();
  int enc2 = _encoder2 ->readRawValue();

  // unwrap encoder values
  enc1 = unwrap(enc1, enc1_prev);
  enc2 = unwrap(enc2, enc2_prev);  

  // calculate new speed
  float speed1_act = (enc1 - enc1_prev)/Ts;
  float speed2_act = (enc2 - enc2_prev)/Ts;

  // shift memories
  for(int i = 1; i > 0; i--){
    ek_speed1[i] = ek_speed1[i-1];
    uk_speed1[i] = uk_speed1[i-1];
    ek_speed2[i] = ek_speed2[i-1];
    uk_speed2[i] = uk_speed2[i-1];
  }

  // calculate new tracking error
  ek_speed1[0] = speed1_des - speed1_act;
  ek_speed2[0] = speed2_des - speed2_act; 

  // compute the new voltages to be sent to the motors
  uk_speed1[0] = 1/(den_contr_speed1[0]) * (-den_contr_speed1[1]*uk_speed1[1] + num_contr_speed1[0]*ek_speed1[0] + num_contr_speed1[1]*ek_speed1[1]);
  uk_speed2[0] = 1/(den_contr_speed2[0]) * (-den_contr_speed2[1]*uk_speed2[1] + num_contr_speed2[0]*ek_speed2[0] + num_contr_speed2[1]*ek_speed2[1]); 

  // clip output of the controllers to allowed voltages
  if(uk_speed1[0] > 6000) { uk_speed1[0] = 6000; }
  if(uk_speed1[0] < -6000) { uk_speed1[0] = -6000; }
  if(uk_speed2[0] > 6000) { uk_speed2[0] = 6000; }
  if(uk_speed2[0] < -6000) { uk_speed2[0] = -6000; }

  // show control signal    (copied to controllerHook())
  // System.setGPoutFloat(0, uk_speed1[0]);
  // System.setGPoutFloat(4, uk_speed2[0]);

  // drive the motors with the calculated values
  _motor1->setBridgeVoltage((int)uk_speed1[0]);
  _motor2->setBridgeVoltage((int)uk_speed2[0]);
  
}
  
int Robot::unwrap(int curr_val, int prev_val){
  /**
   * Unwraps the values of the encoder if needed.
   * Used to avoid sudden changes in encoder values 
   *    due to the limited number of encoder values.
   */
   
  // unwraps two succeeding values
  if (curr_val - prev_val > 32768){
    curr_val = curr_val - 65536;
  }
  else if (curr_val - prev_val < -32768){
    curr_val = curr_val + 65536;
  }
  return curr_val;
}

void Robot::resetVelocityControl()
{
  /**
   * Resets the controller input and output values to zero.
   * This is to avoid weird behaviour, due to transient respons of the wrong input, 
   *    when the controller restarts.
   */
   
    // loop over indices 0-2 to set all registers to zero
    for(int k=0;k<2;k++){
        ek_speed1[k] = 0.0;
        uk_speed1[k] = 0.0;
        ek_speed2[k] = 0.0;
        uk_speed2[k] = 0.0;

        //ek_pos1[k] = 0.0;
        //uk_pos1[k] = 0.0;
        //ek_pos2[k] = 0.0;
        //uk_pos2[k] = 0.0;
    }    
}

