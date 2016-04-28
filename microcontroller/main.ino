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
#include <AccelStepper.h>
#include <MultiStepper.h>

/*****************************************
 *                 DEFINES               *
 *****************************************/

#define BAUD_RATE             9600
#define I2C_ADDRESS           0x04

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
#define MAXSPEED               100
#define SPEED                   50

// I2C COMMANDS
#define STOP                     0
#define PEN_UP                   1
#define PEN_DOWN                 2
#define CALIBRATE                3
#define DRAW                     4

#define CALIBRATE_STEPS         30

#define START_POS              500

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
  bool inCommand;
  bool stepsLeft;
  int nextCommand;
} data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

void i2c_rec_handler(int numBytesReceived);
void i2c_req_handler();

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
/*
boolean moveMotor1(int x);
boolean moveMotor2(int x);
boolean moveMotor3(int x);
boolean moveMotor4(int x);
*/

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

MultiStepper steppers();

long new_positions[4];

Servo penServo;

/*****************************************
 *          HELPER FUNCTIONS             *
 *****************************************/

/**
  * I2C handler called when a message is received
  */
void i2c_rec_handler(int numBytesReceived)
{
    int buf [9];
    int i = 0;
    
    while(Wire.available()) {
        buf[i] = Wire.read();
        i++;
    }
    new_positions[0] = buf[0] * 256 + buf[1];
    new_positions[1] = buf[2] * 256 + buf[3];
    new_positions[2] = buf[4] * 256 + buf[5];
    new_positions[3] = buf[6] * 256 + buf[7];
    data.nextCommand = buf[8];
    Serial.println(" ");
    Serial.print("I2C received : ");
    Serial.print(new_positions[0]);
    Serial.print("\t");    
    Serial.print(new_positions[1]);
    Serial.print("\t");
    Serial.print(new_positions[2]);
    Serial.print("\t");
    Serial.print(new_positions[3]);
    Serial.print("\t");
    Serial.print(buf[8]);
    data.i2cReceived = true;
}

void i2c_req_handler()
{
    Wire.write(data.inCommand);
}

/**
  * Check for transition from calibration to drawing
  */
inline bool allowTransitionCalibrateToDrawing(void)
{
    int c = data.nextCommand;
    bool draw = (c == DRAW) || (c == PEN_UP) || (c == PEN_DOWN);
    return !data.inCommand && draw;     
}

/**
  * Check for transition from drawing to calibration
  */
inline bool allowTransitionDrawingToCalibrate(void)
{
  return !data.inCommand && data.nextCommand == CALIBRATE;
}

inline bool allowTransitionWaitToCalibrate(void)
{
  return !data.inCommand && data.nextCommand == CALIBRATE;
}

inline bool allowTransitionWaitToDrawing(void)
{
    int c = data.nextCommand;
    bool draw = (c == DRAW) || (c == PEN_UP) || (c == PEN_DOWN);
    return !data.inCommand && draw; 
}

inline bool allowTransitionDrawingToWait(void)
{
  return data.nextCommand == STOP;
}

inline bool allowTransitionCalibrateToWait(void)
{
  return data.nextCommand == STOP;
}

/*
 * Process inputs
 */
void processData(void)
{
    if(data.i2cReceived) {
        data.i2cReceived = false;
        data.inCommand = true;
    }
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
            data.presentState = STATE_WAIT;
            Serial.println("WAIT");
        }
        data.inCommand = false;
        break;

      case STATE_CALIBRATE:
        if(data.stateTransition)
        {
            data.presentState = STATE_CALIBRATE;
            Serial.println("CALIBRATE");
            
            new_positions[0] = stepper_1.currentPosition() + CALIBRATE_STEPS;
            new_positions[1] = stepper_2.currentPosition() + CALIBRATE_STEPS;
            new_positions[2] = stepper_3.currentPosition() + CALIBRATE_STEPS;
            new_positions[2] = stepper_4.currentPosition() + CALIBRATE_STEPS;            
            steppers.moveTo(new_positions);
        }
        data.stepsLeft = steppers.run();
        data.inCommand = data.stepsLeft;
        
        if(!data.stepsLeft){
            stepper_1.setCurrentPosition(START_POS);
            stepper_2.setCurrentPosition(START_POS);
            stepper_3.setCurrentPosition(START_POS);
            stepper_4.setCurrentPosition(START_POS);
            Serial.println("CALIBRATED");
        } else {
            Serial.println("STEP");
        }
        break;

      case STATE_DRAWING:
        if(data.stateTransition)
        {
            data.presentState = STATE_DRAWING;
            if(data.nextCommand == DRAW) {
                Serial.println("DRAW");
            } else if(data.nextCommand == PEN_UP) {
                movePenUp();
                data.inCommand = false;
            } else if(data.nextCommand == PEN_DOWN) {
                movePenDown();
                data.inCommand = false;
            }
        }
        if(data.nextCommand == DRAW) {
            
        }
        break;

      default:
        // should never reach here, error
        break;
    }

    data.presentState = state;
}

/*
boolean moveMotor1(int x) {
    if (stepper_1.distanceToGo() == 0)
      { 
          stepper_1.moveTo(x % STEPS);
          stepper_1.setSpeed(MAXSPEED);
      }
    return stepper_1.run();
}

boolean moveMotor2(int x) {
    if (stepper_2.distanceToGo() == 0)
      { 
          stepper_2.moveTo(x % STEPS);
          stepper_2.setSpeed(MAXSPEED);
      }
    return stepper_2.run();
}

boolean moveMotor3(int x) {
    if (stepper_3.distanceToGo() == 0)
      { 
          stepper_3.moveTo(x % STEPS);
          stepper_3.setSpeed(MAXSPEED);
      }
    return stepper_3.run();
}

boolean moveMotor4(int x) {
    if (stepper_4.distanceToGo() == 0)
      { 
          stepper_4.moveTo(x % STEPS);
          stepper_4.setSpeed(MAXSPEED);
      }
    return stepper_4.run();
}
*/

void movePenUp() {
    Serial.println("PEN UP");
    penServo.write(0);
}

void movePenDown() {
    Serial.println("PEN DOWN");
    penServo.write(180);
}

/*****************************************
 *             MAIN FUNCTION             *
 *****************************************/

void setup(void)
{
  Serial.begin(BAUD_RATE);
  Serial.println("-> Starting GhostWriter");

  // Setup i2c communication
  data.i2cReceived = false;
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2c_rec_handler);
  Wire.onRequest(i2c_req_handler);
  /*
   * SETUP MOTOR DIRECTION AND STEP PINS AS OUTPUTS
   */
  stepper_1.setMaxSpeed(MAXSPEED);
  stepper_2.setMaxSpeed(MAXSPEED);
  stepper_3.setMaxSpeed(MAXSPEED);
  stepper_4.setMaxSpeed(MAXSPEED);  
  
  steppers.addStepper(stepper_1);
  steppers.addStepper(stepper_2);
  steppers.addStepper(stepper_3);
  steppers.addStepper(stepper_4);  
  
  penServo.attach(PIN_SERVO);

  data.nextCommand = STOP;
  data.presentState = STATE_WAIT;
  data.desiredState = STATE_WAIT;
  data.stepsLeft = false;
}

void loop(void)
{
  processData();

  getDesiredState();

  data.stateTransition = (data.desiredState != data.presentState);

  setCurrentState();
  
}
