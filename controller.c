// Run a A4998 Stepstick from an Arduino UNOusing AccelStepper
// Paul Hurley Aug 2015
// M_1 Direction pin is 4 step is pin 5
// M_2 D is 25 S is 26
// M_3 D is 6 S is 7
// M_4 D is 23 S is 24
#include
AccelStepper stepper_1(1,5,4);//initialise accelstepper for a two wire board, pin 5 step, pin 4 dir
AccelStepper stepper_2(1,25,26);//initialise accelstepper for a two wire board, pin 26 step, pin 25 dir
AccelStepper stepper_3(1,6,7);//initialise accelstepper for a two wire board, pin 7 step, pin 6 dir
AccelStepper stepper_4(1,23,24);//initialise accelstepper for a two wire board, pin 23 step, pin 23 dir
int currentpos_1;currentpos_2;currentpos_3;currentpos_4;

void setup() {
  Serial.begin(9600);
  pinMode(6,OUTPUT); // Enable
  digitalWrite(6,LOW); // Set Enable low
}

void draw(int[][] commands){
  for(i = 0; i < sizeof(commands)/ sizeof(int); i++){
    moveMotor1(commands[i][0]);
    moveMotor2(commands[i][1]);
    moveMotor2(commands[i][2]);
    moveMotor2(commands[i][3]);
  }
}

void moveMotor1(int x) {
  digitalWrite(6,LOW); // Set Enable low
  if (stepper_1.distanceToGo() == 0)
  {  // Random change to speed, position and acceleration
    // Make sure we dont get 0 speed or accelerations
    delay(1000);
    stepper_1.moveTo(x % 400);
    stepper_1.setMaxSpeed((rand() % 400) + 200);
    stepper_1.setAcceleration((rand() % 200) + 100);
  }

Serial.println(stepper_1.distanceToGo());
stepper_1.run();  // Actually makes stepper move
}

void moveMotor2(int x) {
  digitalWrite(6,LOW); // Set Enable low
  if (stepper_2.distanceToGo() == 0)
  {  // Random change to speed, position and acceleration
    // Make sure we dont get 0 speed or accelerations
    delay(1000);
    stepper_2.moveTo(x % 400);
    stepper_2.setMaxSpeed((rand() % 400) + 200);
    stepper_2.setAcceleration((rand() % 200) + 100);
  }

Serial.println(stepper_2.distanceToGo());
stepper_2.run();  // Actually makes stepper move
}

void moveMotor3(int x) {
  digitalWrite(6,LOW); // Set Enable low
  if (stepper_3.distanceToGo() == 0)
  {  // Random change to speed, position and acceleration
    // Make sure we dont get 0 speed or accelerations
    delay(1000);
    stepper_3.moveTo(x % 400);
    stepper_3.setMaxSpeed((rand() % 400) + 200);
    stepper_3.setAcceleration((rand() % 200) + 100);
  }

Serial.println(stepper_3.distanceToGo());
stepper_3.run();  // Actually makes stepper move
}

void moveMotor4(int x) {
  digitalWrite(6,LOW); // Set Enable low
  if (stepper_4.distanceToGo() == 0)
  {  // Random change to speed, position and acceleration
    // Make sure we dont get 0 speed or accelerations
    delay(1000);
    stepper_4.moveTo(x % 400);
    stepper_4.setMaxSpeed((rand() % 400) + 200);
    stepper_4.setAcceleration((rand() % 200) + 100);
  }

Serial.println(stepper_4.distanceToGo());
stepper_4.run();  // Actually makes stepper move
}