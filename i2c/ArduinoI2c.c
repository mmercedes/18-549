#include <Wire.h>

int data [4];
int x = 0;

void setup() {                                 

Serial.begin(9600);                        
Wire.begin(0x04);                          
Wire.onReceive(receiveData);               //callback for i2c. Jump to void recieveData() function when pi sends data

}

void loop () {

    delay(100);                            //Delay 0.1 seconds. Something for the arduino to do when it is not inside the reciveData() function. This also might be to prevent data collisions.

}

void receiveData(int byteCount) { 

   while(Wire.available()) {               //Wire.available() returns the number of bytes available for retrieval with Wire.read(). Or it returns TRUE for values >0.
       data[x]=Wire.read();
       x++;
       if (x == 4){
        x = 0;
       }
     }
   }

     Serial.println("----");
     Serial.print(data[0]);
     Serial.print("\t");
     Serial.print(data[1]);
     Serial.print("\t");
     Serial.print(data[2]);
     Serial.print("\t");
     Serial.println(data[3]);
     Serial.print("----");

}