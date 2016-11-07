#include "robot.h"
#include <math.h>

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
#define PENDULUM_PIN	A3
#define LED1_PIN 		34
#define LED2_PIN 		40
#define BATTERY_VOLTAGE	6000


Robot::Robot(uint8_t ID):
	_ID(ID),
	_type(20),
	_encoder1(new EncoderSensor(ENC1_PINA,ENC1_PINB)),
	_encoder2(new EncoderSensor(ENC2_PINA,ENC2_PINB)),
	_distance1(new Sharp41S(DIST1_PIN)),
	_distance2(new Sharp41S(DIST2_PIN)),
	_pendulum(new AnalogSensor(PENDULUM_PIN,12)),
	_motor1(new L293D(MOT1_PIN_IN1,MOT1_PIN_IN2,MOT1_PIN_EN,BATTERY_VOLTAGE)),
	_motor2(new L293D(MOT2_PIN_IN1,MOT2_PIN_IN2,MOT2_PIN_EN,BATTERY_VOLTAGE))
{
	_pendulum->setScale(2.0f*M_PI/1023.0f);
}

  // Data for tutorial session controller
  float Kp = 130000;
  float Ki = 600000;
  float Kd = 2000;
  float Ts = 0.01;    // in s
  
  // initial values voor controller tutorial session
  float e_kmin1 = 0;
  float e_kmin2 = 0;
  float u_kmin1 = 0;
  float u_kmin2 = 0;

  float V_ramp = 0;
  int counter = 0;
  
void Robot::init(){
	//initialize the robot - sort of starting procedure
	resetEncoders();
}

void Robot::controllerHook(){
	//do something that is periodic: reading from sensors, setting the motors, updating variables sent to the pc..
	
	if(controlEnabled()){
		//write the control in here

    //random_excitation();
    random_excitation();

    
//     float set_point1 = System.getGPinFloat(1);
//     float cur_pos1 = _encoder1 -> readRawValue();
//
//     float e_k = set_point1 - cur_pos1;
//
//     float u_k = (1/(Kp + Ki*Ts/2 + Kd*2/Ts)) * (e_k - e_kmin2 - u_kmin1*(Ki*Ts - 4/Ts) - u_kmin2*(-Kp + Ki*Ts/2 + Kd*2/Ts));
//
//     e_kmin2 = e_kmin1;
//     e_kmin1 = e_k;
//
//     u_kmin2 = u_kmin1;
//     u_kmin1 = u_k;
//    
//    _motor1 -> setBridgeVoltage(u_k); // set motor1 voltage to variable u_k
//    System.setGPoutFloat(7 ,e_k); // write value 5.1 to floating point channel 7   ("FloatIn7")
//    System.setGPoutFloat(6 ,u_k); // write value 5.1 to floating point channel 7   ("FloatIn7")
    
	} else {
		//set motor voltage to zero or it will keep on running...
    _motor1 -> setBridgeVoltage(0); // set motor1 voltage to 0 mV , i.e. 0V
    _motor2 -> setBridgeVoltage(0); // set motor2 voltage to 0V
	}
}

void Robot::random_excitation()
{
  float V_rand = ((rand()%2)-0.5)*2*(rand() % 12000 - 6000);
  System.setGPoutInt(2 , V_rand);
  _motor1 -> setBridgeVoltage(V_rand);
  _motor2 -> setBridgeVoltage(V_rand);

  int enc1_value = _encoder1 -> readRawValue ();
  System.setGPoutInt(0, enc1_value);
  int enc2_value = _encoder2 -> readRawValue ();
  System.setGPoutInt(1, enc2_value);
}

void Robot::ramp_input()
{

  if (V_ramp > -6000)
  {
    V_ramp = V_ramp - 1;
    float V_motor = min(-3000, V_ramp);

    System.setGPoutInt(2 , V_motor);
    _motor1 -> setBridgeVoltage(V_motor);
    _motor2 -> setBridgeVoltage(V_motor);
  }
  

  int enc1_value = _encoder1 -> readRawValue ();
  System.setGPoutInt(0, enc1_value);
  int enc2_value = _encoder2 -> readRawValue ();
  System.setGPoutInt(1, enc2_value);
}

void Robot::step_input()
{
  float V_motor = 0;
  if (V_ramp > 1000)
  {
    V_motor = -6000;
  }
  else 
  { 
  V_ramp = V_ramp + 1;
  }

  System.setGPoutInt(2 , V_motor);
  _motor1 -> setBridgeVoltage(V_motor);
  _motor2 -> setBridgeVoltage(V_motor);
  

  int enc1_value = _encoder1 -> readRawValue ();
  System.setGPoutInt(0, enc1_value);
  int enc2_value = _encoder2 -> readRawValue ();
  System.setGPoutInt(1, enc2_value);
}


void Robot::block_input()
{
  float breedte_blokpuls = 500;
  if (counter < breedte_blokpuls)
  {
    float V_motor = 3500;

    System.setGPoutInt(2 , V_motor);
    _motor1 -> setBridgeVoltage(V_motor);
    _motor2 -> setBridgeVoltage(V_motor);
  }

  if (counter >= breedte_blokpuls)
  {
    float V_motor = 6000;
    System.setGPoutInt(2 , V_motor);
    _motor1 -> setBridgeVoltage(V_motor);
    _motor2 -> setBridgeVoltage(V_motor);
  }
 

  counter = counter + 1;

  if (counter >= 2*breedte_blokpuls)
  {
    counter = 0;
  }

  int enc1_value = _encoder1 -> readRawValue ();
  System.setGPoutInt(0, enc1_value);
  int enc2_value = _encoder2 -> readRawValue ();
  System.setGPoutInt(1, enc2_value);
}



void Robot::test()
{
  int v_motor1 = System.getGPinInt(0);    //linkse motor als wagentje rechtop is   EN deze functie verwijst naar IntOut0
    int v_motor2 = System.getGPinInt(1);    //rechtse motor als wagentje rechtop is  EN deze functie verwijst naar IntOut1

    _motor1 -> setBridgeVoltage(v_motor1); // set motor1 voltage to variable v_motor
    _motor2 -> setBridgeVoltage(v_motor2); // set motor2 voltage to -3V


    int enc1_value = _encoder1 -> readRawValue ();
    System.setGPoutInt(1 ,enc1_value); // write value enc1_value to integer channel 1  ("IntIn1")    --> positie bepaling van motor 1, grote integers voor grote resolutie
    //System.setGPoutFloat(7 ,5.1); // write value 5.1 to floating point channel 7   ("FloatIn7")
}

void Robot::resetEncoders()
{
	_encoder1->init();
	_encoder2->init();
}


void Robot::resetPendulum()
{
	_pendulum->setOffset(_pendulum->readRawValue());
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

bool Robot::controlEnabled()
{
	return _button_states[0];
}

void Robot::button1callback()
{
	if(toggleButton(0)){
		System.println("Controller enabled.");
	} else{
		System.println("Controller disabled.");
	}
}

void Robot::button2callback()
{
	toggleButton(1);

	resetEncoders();
	resetPendulum();

	System.println("Reset.");
}
void Robot::button3callback()
{
	toggleButton(2);
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
