
#include <Arduino.h>
#include <AccelStepper.h>

const int M1_STEP = 1;
const int M2_STEP = 26;
const int M3_STEP = 11;
const int M4_STEP = 23;

const int M1_DIR = 4;
const int M2_DIR = 25;
const int M3_DIR = 6;
const int M4_DIR = 23;

const int BAUD_RATE = 9600;

const int DELAY = 500;
const int STEPS = 1000;
const int SPEED = 200;
const int ACCEL = 100;

AccelStepper stepper_1(1,M1_STEP, M1_DIR);
AccelStepper stepper_2(1,M2_STEP, M2_DIR);
AccelStepper stepper_3(1,M3_STEP, M3_DIR);
AccelStepper stepper_4(1,M4_STEP, M4_DIR);

int currentpos_1;
int currentpos_2;
int currentpos_3;
int currentpos_4;

/*
boolean moveMotor3(int x) {
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
*/

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.print("Starting...");
  // pinMode(ENABLE_PIN, OUTPUT); // Enable
  // digitalWrite(ENABLE_PIN, LOW); // Set Enable low
  stepper_3.setMaxSpeed(1000);
  stepper_3.setSpeed(50);  
  delay(3000);
}

int count = 1;

void loop() {
    //    stepper_3.runSpeed();
    /*
    int x = 100;
    boolean stepsLeft = true;
    while(stepsLeft) {
        stepsLeft = moveMotor3(x);
        delay(DELAY);
    }
    */
    count++;
    if(count%2 == 0){
        digitalWrite(M3_STEP, LOW);
    } else {
        digitalWrite(M3_STEP, HIGH);
    }
    delay(20);
    Serial.print("moving");
}
