#include <Wire.h>
int counter = 0;
byte positions[4] = {10,10,10,10};
byte reset[4] = {0,0,0,0};
int input; 
int slave; 
 

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(0xA);
  Wire.write(positions, 4);
  Wire.endTransmission();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    if (Serial.readString() == "STOP"){
        Wire.beginTransmission(0xA);
        Wire.write(reset, 4);
        Wire.endTransmission();
        Serial.println("Resetting");
    }
    
  }

}

