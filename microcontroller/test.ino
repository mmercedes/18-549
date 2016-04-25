#include <Arduino.h>
#include <AccelStepper.h>

const int M1_STEP = 3;
const int M2_STEP = A3;
const int M3_STEP = 5;
const int M4_STEP = A1;

const int M1_DIR = 2;
const int M2_DIR = A2;
const int M3_DIR = 4;
const int M4_DIR = A0;

const int BAUD_RATE = 9600;

const int DELAY = 5;
const int STEPS = 1000;
const int MAXSPEED = 500;
const int SPEED = 200;
const int ACCEL = 50;

AccelStepper stepper_1(1,M1_STEP, M1_DIR);
AccelStepper stepper_2(1,M2_STEP, M2_DIR);
AccelStepper stepper_3(1,M3_STEP, M3_DIR);
AccelStepper stepper_4(1,M4_STEP, M4_DIR);

int currentpos_1;
int currentpos_2;
int currentpos_3;
int currentpos_4;

// 1 and 4 are diagonals
// 2 and 3 as well

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



void setup() {
  Serial.begin(BAUD_RATE);
  Serial.print("Starting...");
  // pinMode(ENABLE_PIN, OUTPUT); // Enable
  // digitalWrite(ENABLE_PIN, LOW); // Set Enable low
  stepper_3.setMaxSpeed(MAXSPEED);
  stepper_3.setSpeed(SPEED);  
  stepper_2.setMaxSpeed(MAXSPEED);
  stepper_2.setSpeed(SPEED);
  delay(1000);
}


void loop() {
    //    stepper_3.runSpeed();
    int x = 200;
    boolean stepsLeft3 = true;
    boolean stepsLeft2 = true;
    
    while(true) {
        while(stepsLeft2 || stepsLeft3) {
            stepsLeft3 = moveMotor3(x);
            stepsLeft2 = moveMotor2(x);
            delay(DELAY);
        }
        x = -x;
        stepsLeft2 = true;
        stepsLeft3 = true;
    }
}
