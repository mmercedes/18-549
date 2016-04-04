// Run a A4998 Stepstick from an Arduino UNOusing AccelStepper
// Paul Hurley Aug 2015
// M_1 Direction pin is 4 step is pin 5
// M_2 D is 25 S is 26
// M_3 D is 6 S is 7
// M_4 D is 23 S is 24

//https://github.com/sudar/Arduino-Makefile
#include <Arduino.h>
#include <AccelStepper.h>

const int M1_STEP = 1;
const int M2_STEP = 26;
const int M3_STEP = 7;
const int M4_STEP = 23;

const int M1_DIR = 4;
const int M2_DIR = 25;
const int M3_DIR = 6;
const int M4_DIR = 23;

const int ENABLE_PIN = 1;
const int BAUD_RATE = 9600;

const int DELAY = 1000;
const int STEPS = 1000;
const int SPEED = 20;
const int ACCEL = 10;

AccelStepper stepper_1(1,M1_STEP, M1_DIR); //initialise accelstepper for a two wire board, pin 5 step, pin 4 dir
AccelStepper stepper_2(1,M2_STEP, M2_DIR); //initialise accelstepper for a two wire board, pin 26 step, pin 25 dir
AccelStepper stepper_3(1,M3_STEP, M3_DIR); //initialise accelstepper for a two wire board, pin 7 step, pin 6 dir
AccelStepper stepper_4(1,M4_STEP, M4_DIR); //initialise accelstepper for a two wire board, pin 23 step, pin 23 dir

int currentpos_1;
int currentpos_2;
int currentpos_3;
int currentpos_4;

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(ENABLE_PIN, OUTPUT); // Enable
  digitalWrite(ENABLE_PIN, LOW); // Set Enable low
}

void draw(int[][] commands){
  for(i = 0; i < sizeof(commands)/ sizeof(int); i++){
      int x1 = commands[i][0];
      int x2 = commands[i][1];
      int x3 = commands[i][2];
      int x4 = commands[i][3];      

      boolean stepsLeft = true;
      while(stepsLeft) {
          stepsLeft = moveMotor1(x1) || moveMotor(x2) || moveMotor(x3) || moveMotor(x4);
          delay(DELAY);
      }
  }
}

void moveMotor1(int x) {
    // digitalWrite(ENABLE_PIN,LOW); // Set Enable low
    if (stepper_1.distanceToGo() == 0)
        {
            stepper_1.moveTo(x % STEPS);
            stepper_1.setMaxSpeed(SPEED);
            stepper_1.setAcceleration(ACCEL);
        }
    Serial.println(stepper_1.distanceToGo());
    return stepper_1.run();
}

void moveMotor2(int x) {
    // digitalWrite(ENABLE_PIN,LOW); // Set Enable low    
    if (stepper_2.distanceToGo() == 0)
        { 
            stepper_2.moveTo(x % STEPS);
            stepper_2.setMaxSpeed(SPEED);
            stepper_2.setAcceleration(ACCEL);
        }
    Serial.println(stepper_2.distanceToGo());
    return stepper_2.run();    
}

void moveMotor3(int x) {
    // digitalWrite(ENABLE_PIN,LOW); // Set Enable low        
    if (stepper_3.distanceToGo() == 0)
      { 
          stepper_3.moveTo(x % STEPS);
          stepper_3.setMaxSpeed(SPEED);
          stepper_3.setAcceleration(ACCEL);
      }
    Serial.println(stepper_3.distanceToGo());
    return stepper_3.run();
}

void moveMotor4(int x) {
    // digitalWrite(ENABLE_PIN,LOW); // Set Enable low            
    if (stepper_4.distanceToGo() == 0)
        {
            stepper_4.moveTo(x % STEPS);
            stepper_4.setMaxSpeed(SPEED);
            stepper_4.setAcceleration(ACCEL);
        }
    Serial.println(stepper_4.distanceToGo());
    return stepper_4.run();
}
