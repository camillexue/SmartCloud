#include <Wire.h>

//i2c addresses for slave arduinos
int slaveA = 0xA;
int slaveB = 0xB;
int slaveC = 0xC;
int slaveD = 0xD;
int slaveE = 0xE;
int slaveF = 0xF;

int slaves[6] = {slaveA, slaveB, slaveC, slaveD, slaveE, slaveF};

//two dimensional array of all positions
byte allPositions[6][4];

int state = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
     state = (int)Serial.read(); //value 1-4
     Serial.println(state);
  }

  switch(state) {
    case 1: //sunny
      sunny();
      break;
    case 2: //partly cloudy
      partcloudy();
      break;
    case 3: //cloudy
      cloudy();
      break;
    case 4: //rainy
      rainy();
      break;
     default: //nothing
      sunny();
      break;
  }

  //send all position messages to all 6 slaves
  
  for(int j = 0; j < 6; j++) {
    //position sent to slave corresponds to column of allPositions
    byte positions[4]; 
    for(int k = 0; k < 4; k++) {
      positions[k] = allPositions[j][k];
    }
    
    //send position message to one slave
    Wire.beginTransmission(slaves[j]);
    Wire.write(positions, 4);
    Wire.endTransmission();
  }
  
}
void reset() {
  for (int j = 0; j < 6; j++) {
    for (int k = 0; k < 4; k++) {
      allPositions[j][k] = 0;
    }
  }
}

void sunny() {

}

void rainy() {

}

void cloudy() {

}

void partcloudy() {

}
