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
#include <Wire.h>

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

} data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

void i2c_handler(int numBytesReceived);

void processData(void);
void getDesiredState(void);
void setCurrentState(void);

// Calibrate transitions
inline bool allowTransitionCalibrateToDrawing(void);
inline bool allowTransitionCalibrateToWait(void);

// Drawing transitions
inline bool allowTransitionDrawingToCalibrate(void);
inline bool allowTransitionDrawingToWait(void);

// Wait transitions
inline bool allowTransitionWaitToDrawing(void);
inline bool allowTransitionWaitToCalibrate(void);


/*****************************************
 *              VARIABLES                *
 *****************************************/

data_S data;

/*****************************************
 *          HELPER FUNCTIONS             *
 *****************************************/

/**
  * I2C handler called when a message is received
  */
void i2c_handler(int numBytesReceived)
{

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

}

/*
 * Get the present state and determine if a state transition shall be performed. If so, set the 
 * desired state to that new state.
 */
void getDesiredState(void)
{
  states_E desiredState = data.desiredState;

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

/*****************************************
 *             MAIN FUNCTION             *
 *****************************************/

void setup(void)
{
  Serial.begin(BAUD_RATE);
  Serial.println("-> Starting GhostWriter");

  // Setup i2c communication
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2c_handler);

  /*
   * SETUP MOTOR DIRECTION AND STEP PINS AS OUTPUTS
   */
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR1_DIR,  OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR,  OUTPUT);
  pinMode(PIN_MOTOR3_STEP, OUTPUT);
  pinMode(PIN_MOTOR3_DIR,  OUTPUT);
  pinMode(PIN_MOTOR4_STEP, OUTPUT);
  pinMode(PIN_MOTOR4_DIR,  OUTPUT);
  pinMode(PIN_SERVO,       OUTPUT);

}

void loop(void)
{

  processData();

  getDesiredState();

  data.stateTransition = (data.desiredState != data.presentState);

  setCurrentState();

  digitalWrite(PIN_MOTOR1_STEP, HIGH);
  digitalWrite(PIN_MOTOR2_STEP, HIGH);
  digitalWrite(PIN_MOTOR3_STEP, HIGH);
  digitalWrite(PIN_MOTOR4_STEP, HIGH);
  delay(5);
  digitalWrite(PIN_MOTOR1_STEP, LOW);
  digitalWrite(PIN_MOTOR2_STEP, LOW);
  digitalWrite(PIN_MOTOR3_STEP, LOW);
  digitalWrite(PIN_MOTOR4_STEP, LOW);
  delay(5);

}
