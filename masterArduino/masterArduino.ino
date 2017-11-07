#include <Wire.h>

int address1 = 9; // address for 
int x = 5;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(address1);
  Wire.send(x);
  Wire.endTransmission();
  delay(500);
}
