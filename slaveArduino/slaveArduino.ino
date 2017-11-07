#include <Wire.h>

int I2C_address = 9;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_address);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void receiveEvent() {
  while(Wire.available()) {
    int x = Wire.read();
    Serial.print(x);
  }
}

