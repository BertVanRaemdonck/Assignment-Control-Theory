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
  
void Robot::init(){
	//initialize the robot - sort of starting procedure
	resetEncoders();
  // reset controller so that it does not do crazy things when we restart it
  reset_controller();
  counter = 0.0;
}

void Robot::controllerHook(){
	//do something that is periodic: reading from sensors, setting the motors, updating variables sent to the pc..
  
	if(controlEnabled()){
		//write the control in here
   float speed1_des = System.getGPinFloat(0);
   float speed2_des = System.getGPinFloat(1);
   int enc1_curr = _encoder1 ->readRawValue();
   int enc2_curr = _encoder2 ->readRawValue();
   float pos1_des = System.getGPinFloat(2);
   float pos2_des = System.getGPinFloat(3);
   
   controller_position(pos1_des, pos2_des);
   
   System.setGPoutFloat(0, speed1_des);
   System.setGPoutFloat(1,(enc1_curr-enc1_prev)/Ts);
   System.setGPoutFloat(2, pos1_des);
   System.setGPoutFloat(3, enc1_curr);
   System.setGPoutFloat(4, speed2_des);
   System.setGPoutFloat(5, (enc2_curr-enc2_prev)/Ts);
   System.setGPoutFloat(6, pos2_des);
   System.setGPoutFloat(7, enc2_curr);
   
   enc1_prev = enc1_curr;
   enc2_prev = enc2_curr;

   System.setGPoutInt(0, enc1_curr - enc2_curr);
    
	} else {
		//set motor voltage to zero or it will keep on running...
    _motor1 -> setBridgeVoltage(0); // set motor1 voltage to 0 mV , i.e. 0V
    _motor2 -> setBridgeVoltage(0); // set motor2 voltage to 0V
    // reset controller so that it does not do crazy things when we restart it
    reset_controller();
    counter = 0.0;
	}
}


void Robot::controller_position(float pos1_des, float pos2_des)
{
  /**
   * This is the implemented position controller.
   * This can be used to control the position of the motors seperatly.
   */

  // read encoder values
  int enc1 = _encoder1 ->readRawValue();
  int enc2 = _encoder2 ->readRawValue();

  // unwrap encoder values
  enc1 = unwrap(enc1, enc1_prev);
  enc2 = unwrap(enc2, enc2_prev);
    
  // shift memories
  for(int i = 1; i > 0; i--){
    ek_pos1[i] = ek_pos1[i-1];
    uk_pos1[i] = uk_pos1[i-1];
    ek_pos2[i] = ek_pos2[i-1];
    uk_pos2[i] = uk_pos2[i-1];
  }   

  // calculate new tracking error
  ek_pos1[0] = pos1_des - enc1;
  ek_pos2[0] = pos2_des - enc2;

  // implementing proportional controller
  if (sel_pos_contr == 0) {
    uk_pos1[0] = 1/(den_Pcontr_pos1[0]) * (num_Pcontr_pos1[0]*ek_pos1[0]);
    uk_pos2[0] = 1/(den_Pcontr_pos1[0]) * (num_Pcontr_pos1[0]*ek_pos1[0]);
  } else {
    uk_pos1[0] = 1/(den_PIcontr_pos1[0]) * (-den_PIcontr_pos1[1]*uk_pos1[1] + num_PIcontr_pos1[0]*ek_pos1[0] + num_PIcontr_pos1[1]*ek_pos1[1]);
    uk_pos2[0] = 1/(den_PIcontr_pos2[0]) * (-den_PIcontr_pos2[1]*uk_pos2[1] + num_PIcontr_pos2[0]*ek_pos2[0] + num_PIcontr_pos2[1]*ek_pos2[1]);
  }

  // calculate desired speed
  float speed1_des = (ek_pos1[0] - ek_pos1[1])/Ts;
  float speed2_des = (ek_pos2[0] - ek_pos2[1])/Ts;

  // using speed controller
  controller_speed(speed1_des, speed2_des);
  
}
  


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

  // show control signal
  System.setGPoutFloat(2, uk_speed1[0]);
  System.setGPoutFloat(6, uk_speed2[0]);

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

void Robot::reset_controller()
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

        ek_pos1[k] = 0.0;
        uk_pos1[k] = 0.0;
        ek_pos2[k] = 0.0;
        uk_pos2[k] = 0.0;
    }
    
}


void Robot::random_excitation(int period)
{
  /**
   * Writes a periodic random voltage signal to both motors.
   * If period = 0, the signal has no periodicity.
   * Used for the identification of the motors.
   */
   
  if (period > 0)
  {
    if (counter >= period)
    {
      srand(1);
      counter = 0;
    }
    else
    {
      counter = counter + 1;
    }
  }
   
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
  /**
   * Writes a ramp voltage signal to both motors.
   * Used for the identification of the motors.
   */
   
  if (counter > -6000)
  {
    counter = counter - 1;
    float V_motor = min(-3000, counter);

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
  /**
   * Writes a step voltage signal to both motors.
   * Used for the identification of the motors.
   */
  
  float V_motor = 0;
  if (counter > 1000)
  {
    V_motor = -6000;
  }
  else 
  { 
    counter = counter + 1;
  }

  System.setGPoutInt(2 , V_motor);
  _motor1 -> setBridgeVoltage(V_motor);
  _motor2 -> setBridgeVoltage(V_motor);
  

  int enc1_value = _encoder1 -> readRawValue ();
  System.setGPoutInt(0, enc1_value);
  int enc2_value = _encoder2 -> readRawValue ();
  System.setGPoutInt(1, enc2_value);
}

float Robot::block_input_reference(float width_block, float val_low, float val_high)
{
  /*
   * Returns a float following a block signal every time it's called. The block starts
   * on val_low and goes to val_high, and has a width of width_block. The signal is
   * periodical.
   */
   
   float ref;
   if (counter < width_block)
   {
    ref = val_low;
   }
   else if (counter >= width_block)
   {
    ref = val_high;
   }
   counter += 1;

   if (counter >= 2*width_block)
   {
    counter = 0.0;
   }

   return ref;
}

void Robot::block_input_excitation(float breedte_blokpuls)
{
  /**
   * Writes a periodic block input voltage signal to both motors.
   * Used for the identification of the motors.
   */
  
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
  /**
   * Writes a voltage signal to the motors based on input from QRobotics center.
   * Used for initial tests on the cart.
   */
   
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
