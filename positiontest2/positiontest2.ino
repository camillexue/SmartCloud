#include <Wire.h>

void changeSingle(int x, int y, int pos);

//i2c addresses for slave arduinos
int slaveA = 0x0A;
int slaveB = 0x0B;
int slaveC = 0x0C;
int slaveD = 0x0D;
int slaveE = 0x0E;
int slaveF = 0x0F;

int arduinoNumber = 0;
int motorNumber = 0;
int address = 0;
int slaves[6] = {slaveA, slaveB, slaveC, slaveD, slaveE, slaveF};

//two dimensional array of all positions
byte allPositions[6][4];

int motorAddresses[6][4] = {{12, 11, 22, 21}, 
                            {14, 13, 23, 24}, 
                            {32, 31, 42, 41}, 
                            {34, 33, 43, 44},
                            {52, 51, 62, 61},
                            {54, 53, 63, 64}};

int x,y,pos;

byte positions[4] = {0,0,0,0};

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  moveToFive();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String input = Serial.readString();
    if (input == "SET") {
      Serial.print("Enter X Coordinate:");
      x = getValue();
      Serial.println(x);
      Serial.print("Enter Y Coordinate:");
      y = getValue();
      Serial.println(y);
      Serial.print("Enter Position Value: ");
      pos = getValue();
      Serial.println(pos);
      changeSingle(x, y, pos);
    }
    else if (input == "RESET") {
      reset();
      Serial.println("Resetting");
     }
  }
  delay(200);
  //updateAll();
}
  

void reset() {
  for (int j = 0; j < 6; j++) {
    for (int k = 0; k < 4; k++) {
      allPositions[j][k] = 0;
    }
  }
  updateAll();
}

void moveToFive() {
  for (int j = 0; j < 6; j++) {
    for (int k = 0; k < 4; k++) {
      allPositions[j][k] = 5;
    }
  }
  updateAll();
}

void updateAll() {
  byte positions[6][4];
  for(int j = 0; j < 6; j++) { 
      for (int k = 0; k < 4; k++) {
        address = motorAddresses[j][k];
        motorNumber = address % 10;
        arduinoNumber = (address - motorNumber) / 10;
        positions[arduinoNumber-1][motorNumber] = allPositions[j][k];
      }
    }
    byte specificPositions[4];
    for(int x = 0; x < 1; x++){
      for (int y = 0; y < 4; y++){
        specificPositions[y]=positions[x][y];
      }
      Wire.beginTransmission(slaves[x]);
      Wire.write(specificPositions, 4);
      Wire.endTransmission();     
    }

}

void changeSingle(int x,int y,int pos) {
  
  int changeAddress = motorAddresses[x][y];
  int changeMotorNumber = changeAddress % 10;
  int changeArduinoNumber = (changeAddress - changeMotorNumber) / 10;
  
  for (int k = 0; k < 4; k++) {
    int address = motorAddresses[x][k];
    motorNumber = address % 10;
    positions[k] = allPositions[x][k];
    arduinoNumber = (address - motorNumber) / 10;
   }

  positions[changeMotorNumber - 1] = pos;
  allPositions[x][y] = pos;
  
  Wire.beginTransmission(slaves[changeArduinoNumber -1]);
  Wire.write(positions, 4);
  Wire.endTransmission();

//  Serial.print(float(positions[0]));
//  Serial.print(" ");
//  Serial.print(float(positions[1]));
//  Serial.print(" ");
//  Serial.print(float(positions[2]));
//  Serial.print(" ");
//  Serial.println(float(positions[3]));
  
  Serial.print("Changed Arduino ");
  Serial.print(changeArduinoNumber);
  Serial.print(" motor ");
  Serial.print(changeMotorNumber);
  Serial.print(" to ");
  Serial.println(pos);
  Serial.println();
}

int getValue() {
  int value = -1;
  while(value == -1) {
    if (Serial.available() > 0) {
      value = Serial.parseInt();
    }
  }
  return value;
}
