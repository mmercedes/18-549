#include <Arduino.h>
#include <Wire.h>

#define SCL_PIN 28
#define SCL_PORT PORTD
#define SDA_PIN 27
#define SDA_PORT PORTC

int data [4];
int x = 0;
int y = 0;

void receiveData(int byteCount) {
    
   while(Wire.available()) {               //Wire.available() returns the number of bytes available for retrieval with Wire.read(). Or it returns TRUE for values >0.
       y = Wire.read();
       Serial.println(y);
       //data[x]=Wire.read();
       //x++;
   }
}

void setup() {                                 

Serial.begin(9600);                        
Wire.begin(0x04);                          
Wire.onReceive(receiveData);               //callback for i2c. Jump to void recieveData() function when pi sends data
}
 
void loop () {
    delay(100);                            //Delay 0.1 seconds. Something for the arduino to do when it is not inside the reciveData() function. This also might be to prevent data collisions.
}

