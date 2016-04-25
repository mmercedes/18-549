#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int data [9];
int number = 0;
int state = 0;
int x = 0;
int a = 0;
int b = 0;
int c = 0;
int d = 0;
int s = 0;

void setup() {
Serial.begin(9600); // start serial for output
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
delay(100);
}

// callback for received data
void receiveData(int byteCount){
  Serial.println("here");
  while(Wire.available()) {
    data[x] = Wire.read();
    x ++;
  
    }
    a = data[0] * 256 + data[1];
    b = data[2] * 256 + data[3];
    c = data[4] * 256 + data[5];
    d = data[6] * 256 + data[7];
    s = data[8];
    Serial.print("data received: ");
    Serial.print(a);
    Serial.print("\t");
    Serial.print(b);
    Serial.print("\t");
    Serial.print(c);
    Serial.print("\t");
    Serial.print(d);
    Serial.print("\t");
    Serial.print(s);
    Serial.print("\t");
    x = 0;
}

// callback for sending data
void sendData(){
  if (s == 1){
    Wire.write(12);
  }
}
