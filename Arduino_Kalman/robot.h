#ifndef ROBOT_H
#define ROBOT_H

//!  ROBOT Class
/*!
  Class incorporating the robot. This class is used to define state machines, control algorithms, sensor readings,...
  It should be interfaced with the communicator to send data to the world.
*/

#include <inttypes.h>
#include "math.h"

#include <microOS.h>
#include <encoder_sensor.h>
#include <difference_sensor.h>
#include <sharp41S.h>
#include <l293d.h>
#include <joystick.h>
#include "ekf_cart.h"
#include "navigation_controller.h"

class Robot
{
  private:
    uint8_t _ID;			//give the robot an ID so that you can recognize it
    uint8_t _type;

    // Give the robot some sensors
    Sensor1D* _encoder1;
    Sensor1D* _encoder2;
    Sensor1D* _speed1;
    Sensor1D* _speed2;
    Sensor1D* _distance1;
    Sensor1D* _distance2;

    // Give the robot some motors
    HBridgeInterface* _motor1;
    HBridgeInterface* _motor2;

    // Interface the buttons
    bool _button_states[8] = {false, false, false, false, false, false, false, false};
    bool toggleButton(uint8_t button);
    bool KalmanFilterEnabled();
    bool navigationEnabled();

    // Kalman filter
    EkfCart _ekf;

    // Navigation controller
    NavigationController _nav;

  public:
    Robot(uint8_t ID = 0);

    ////////
    /// FUNC
    void init();			//set up the robot
    void controllerHook();	//update function which can be executed continuously
    void resetEncoders();	//reset the encoders
    double wheelSpeedA();
    double wheelSpeedB();

    void resetKalmanFilter();
    void resetNavigationController();

    void velocityControlUpdate(double setpoint_left, double setpoint_right); //update motor speed

    double wrap2pi(double angle);

    ///////
    /// SET
    void setID(uint8_t ID);

    ///////
    /// GET
    uint8_t id();
    uint8_t type();

    // Event callbacks
    void button1callback();
    void button2callback();
    void button3callback();
    void button4callback();
    void button5callback();
    void button6callback();
    void button7callback();
    void button8callback();
};

#endif //ROBOT_H
