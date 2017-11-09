#include <Wire.h>

int address1 = 9; // address for 
int x = 5;
int state;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
     state = (int)Serial.read(); //value 1-4
     Serial.println(state);
  }
  
  Wire.beginTransmission(address1);
  Wire.write(state);
  Wire.endTransmission();
  delay(500);
}
