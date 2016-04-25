/**
  * File: main.c
  * Authors: Lucas Bruder, Matt Mercedes, and Jake Stephens
  * Class: 18-549 Embedded System Design
  * Team 21 - GhostWriter
  * Last Modified: 4/17/2016
  *
  * This file contains the code to be uploaded on the Atmega328p on the PCB
  *
  *
  */

/*****************************************
 *                 INCLUDES              *
 *****************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

/*****************************************
 *                 DEFINES               *
 *****************************************/

#define BAUD_RATE           115200
#define I2C_ADDRESS              1

#define PIN_MOTOR1_STEP          3
#define PIN_MOTOR1_DIR           2
#define PIN_MOTOR2_STEP         A3
#define PIN_MOTOR2_DIR          A2
#define PIN_MOTOR3_STEP          5
#define PIN_MOTOR3_DIR           4
#define PIN_MOTOR4_STEP         A1
#define PIN_MOTOR4_DIR          A0
#define PIN_SERVO                7

#define DELAY                    5
#define STEPS                 1000
#define MAXSPEED               500
#define SPEED                  200
#define ACCEL                   50

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/

typedef enum
{
  STATE_WAIT,
  STATE_CALIBRATE,
  STATE_DRAWING,
} states_E;

typedef struct
{
  states_E desiredState;
  states_E presentState;
  bool stateTransition;
  bool i2cReceived;
} data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

void i2c_handler(int numBytesReceived);

void processData(void);
void getDesiredState(void);
void setCurrentState(void);

void setup(void);
void loop(void);

// Calibrate transitions
inline bool allowTransitionCalibrateToDrawing(void);
inline bool allowTransitionCalibrateToWait(void);

// Drawing transitions
inline bool allowTransitionDrawingToCalibrate(void);
inline bool allowTransitionDrawingToWait(void);

// Wait transitions
inline bool allowTransitionWaitToDrawing(void);
inline bool allowTransitionWaitToCalibrate(void);

// Motor movement functions
boolean moveMotor1(int x);
boolean moveMotor2(int x);
boolean moveMotor3(int x);
boolean moveMotor4(int x);

void movePenUp();
void movePenDown();

/*****************************************
 *              VARIABLES                *
 *****************************************/

data_S data;

AccelStepper stepper_1(1,PIN_MOTOR1_STEP, PIN_MOTOR1_DIR);
AccelStepper stepper_2(1,PIN_MOTOR2_STEP, PIN_MOTOR2_DIR);
AccelStepper stepper_3(1,PIN_MOTOR3_STEP, PIN_MOTOR3_DIR);
AccelStepper stepper_4(1,PIN_MOTOR4_STEP, PIN_MOTOR4_DIR);

int currentpos_1;
int currentpos_2;
int currentpos_3;
int currentpos_4;

int i2cmsg [4];

/*****************************************
 *          HELPER FUNCTIONS             *
 *****************************************/

/**
  * I2C handler called when a message is received
  */
void i2c_handler(int numBytesReceived)
{
    int i = 0;

    while(Wire.available()) {
        i2cmsg[i] = Wire.read();
        Serial.print("I2C received : ");
        Serial.println(i2cmsg[i], DEC);
        i++;
    }

    data.i2cReceived = true;
}

/**
  * Check for transition from calibration to drawing
  */
inline bool allowTransitionCalibrateToDrawing(void)
{
  bool allowTransition = false;

  return allowTransition;
}

/**
  * Check for transition from drawing to calibration
  */
inline bool allowTransitionDrawingToCalibrate(void)
{
  bool allowTransition = false;

  return allowTransition;
}

inline bool allowTransitionWaitToCalibrate(void)
{
  bool allowTransition = false;

  return allowTransition;
}

inline bool allowTransitionWaitToDrawing(void)
{
  bool allowTransition = false;

  return allowTransition;
}

inline bool allowTransitionDrawingToWait(void)
{
  bool allowTransition = false;

  return allowTransition;
}

inline bool allowTransitionCalibrateToWait(void)
{
  bool allowTransition = false;

  return allowTransition;
}

/*
 * Process inputs
 */
void processData(void)
{
    if(data.i2cReceived) {
        data.i2cReceived = false;
    }
}

/*
 * Get the present state and determine if a state transition shall be performed. If so, set the 
 * desired state to that new state.
 */
void getDesiredState(void)
{
  states_E desiredState = data.desiredState;
v
  switch(desiredState)
  {
    case STATE_WAIT:
      if(allowTransitionWaitToCalibrate() == true)
      {
        desiredState = STATE_CALIBRATE;
      }
      else if(allowTransitionWaitToDrawing() == true)
      {
        desiredState = STATE_DRAWING;
      }
      else
      {
        // keep state
      }

      break;

    case STATE_CALIBRATE:
      if(allowTransitionCalibrateToDrawing() == true)
      {
        desiredState = STATE_DRAWING;
      }
      else if(allowTransitionCalibrateToWait() == true)
      {
        desiredState = STATE_WAIT;
      }
      else
      {
        // keep state
      }
      break;

    case STATE_DRAWING:
      if(allowTransitionDrawingToCalibrate() == true)
      {
        desiredState = STATE_CALIBRATE;
      }
      else if(allowTransitionDrawingToWait() == true)
      {
        desiredState = STATE_WAIT;
      }
      else
      {
        // keep state
      }
      break;

    default:
      // should never reach here, error
      break;
  }
    
  data.desiredState = desiredState;
}

/*
 * Perform any transition and state related outputs
 */
void setCurrentState(void)
{
    states_E state = data.desiredState;

    switch(state)
    {
      case STATE_WAIT:
        if(data.stateTransition)
        {

        }

        break;

      case STATE_CALIBRATE:
        if(data.stateTransition)
        {

        }

        break;

      case STATE_DRAWING:
        if(data.stateTransition)
        {

        }

        break;

      default:
        // should never reach here, error
        break;
    }

    data.presentState = state;
}


boolean moveMotor1(int x) {
    if (stepper_1.distanceToGo() == 0)
      { 
          stepper_1.moveTo(x % STEPS);
          stepper_1.setMaxSpeed(MAXSPEED);
          stepper_1.setAcceleration(ACCEL);
      }
    return stepper_1.run();
}

boolean moveMotor2(int x) {
    if (stepper_2.distanceToGo() == 0)
      { 
          stepper_2.moveTo(x % STEPS);
          stepper_2.setMaxSpeed(MAXSPEED);
          stepper_2.setAcceleration(ACCEL);
      }
    return stepper_2.run();
}

boolean moveMotor3(int x) {
    if (stepper_3.distanceToGo() == 0)
      { 
          stepper_3.moveTo(x % STEPS);
          stepper_3.setMaxSpeed(MAXSPEED);
          stepper_3.setAcceleration(ACCEL);
      }
    return stepper_3.run();
}

boolean moveMotor4(int x) {
    if (stepper_4.distanceToGo() == 0)
      { 
          stepper_4.moveTo(x % STEPS);
          stepper_4.setMaxSpeed(MAXSPEED);
          stepper_4.setAcceleration(ACCEL);
      }
    return stepper_4.run();
}

void movePenUp() {

}

void movePenDown() {

}

/*****************************************
 *             MAIN FUNCTION             *
 *****************************************/

void setup(void)
{
  Serial.begin(BAUD_RATE);
  Serial.println("-> Starting GhostWriter");

  // Setup i2c communication
  data.i2cRecieved = false;
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2c_handler);

  /*
   * SETUP MOTOR DIRECTION AND STEP PINS AS OUTPUTS
   */
  stepper_1.setMaxSpeed(MAXSPEED);
  stepper_1.setSpeed(SPEED);  
  stepper_2.setMaxSpeed(MAXSPEED);
  stepper_2.setSpeed(SPEED);
  stepper_3.setMaxSpeed(MAXSPEED);
  stepper_3.setSpeed(SPEED);
  stepper_4.setMaxSpeed(MAXSPEED);
  stepper_4.setSpeed(SPEED);
  
  pinMode(PIN_SERVO,       OUTPUT);

}

void loop(void)
{

  processData();

  getDesiredState();

  data.stateTransition = (data.desiredState != data.presentState);

  setCurrentState();
}
