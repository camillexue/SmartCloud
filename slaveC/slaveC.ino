#include <Wire.h>

int I2C_address = 0x0C;

#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "UnipolarStepper.h"

byte positions[4] = {0, 0, 0, 0};
byte positionBuffer[4] = {0, 0, 0, 0};
bool newPosition = false;
// pin assignments

const int MOTOR1_IN1 = A3;
const int MOTOR1_IN2 = 2;
const int MOTOR1_IN3 = 13;
const int MOTOR1_IN4 = 3;

const int MOTOR2_IN1 = 4;
const int MOTOR2_IN2 = 5;
const int MOTOR2_IN3 = 6;
const int MOTOR2_IN4 = 7;

const int MOTOR3_IN1 = A0;
const int MOTOR3_IN2 = A1;
const int MOTOR3_IN3 = 8;
const int MOTOR3_IN4 = A2;

const int MOTOR4_IN1 = 12;
const int MOTOR4_IN2 = 11;
const int MOTOR4_IN3 = 10;
const int MOTOR4_IN4 = 9;

const int FULL_STEPS_PER_REVOLUTION = 2048;


// create the stepper motor object

UnipolarStepper stepper1;
UnipolarStepper stepper2;
UnipolarStepper stepper3;
UnipolarStepper stepper4;


void setup() {
  Wire.begin(I2C_address);
  Wire.onReceive(receiveEvent);

  stepper1.connectToPins(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4);
  stepper2.connectToPins(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4);
  stepper3.connectToPins(MOTOR3_IN1, MOTOR3_IN2, MOTOR3_IN3, MOTOR3_IN4);
  stepper4.connectToPins(MOTOR4_IN1, MOTOR4_IN2, MOTOR4_IN3, MOTOR4_IN4);
  Serial.begin(9600); 
  delay(300);
}

void loop() {
  
  // set the number of steps per revolutions, 4096 is typical for a 28BYJ-48

  stepper1.setStepsPerRevolution(4096);
  stepper2.setStepsPerRevolution(4096);
  stepper3.setStepsPerRevolution(4096);
  stepper4.setStepsPerRevolution(4096);

  // set the speed in rotations/second and acceleration in rotations/second/second

  stepper1.setSpeedInRevolutionsPerSecond(.4);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper2.setSpeedInRevolutionsPerSecond(.4);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper3.setSpeedInRevolutionsPerSecond(.4);
  stepper3.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper4.setSpeedInRevolutionsPerSecond(.4);
  stepper4.setAccelerationInRevolutionsPerSecondPerSecond(.2);

  // setup motor 1 to move forward 4.0 revolutions

  Serial.print(float(positions[0]));
  Serial.print(" ");
  Serial.print(float(positions[1]));
  Serial.print(" ");
  Serial.print(float(positions[2]));
  Serial.print(" ");
  Serial.println(float(positions[3])); 
  
  stepper1.setupMoveInRevolutions(float(positions[0]));
  stepper2.setupMoveInRevolutions(float(positions[1]));
  stepper3.setupMoveInRevolutions(float(positions[2]));
  stepper4.setupMoveInRevolutions(float(positions[3]));
  
  // execute the moves
  while((!stepper1.motionComplete()) || (!stepper2.motionComplete()) || (!stepper3.motionComplete()) || (!stepper4.motionComplete()))
  {
    stepper1.processMovement();
    stepper2.processMovement();
    stepper3.processMovement();
    stepper4.processMovement();
  }
}


void receiveEvent(int howMany) {
  while(Wire.available() == 4) {
    Wire.readBytes(positionBuffer,4);
  }
//  Serial.print(float(positions[0]));
//  Serial.print(" ");
//  Serial.print(float(positions[1]));
//  Serial.print(" ");
//  Serial.print(float(positions[2]));
//  Serial.print(" ");
//  Serial.println(float(positions[3])); 
  for (int k = 0; k < 4; k++) {
    positions[k] = positionBuffer[k];
  }
}

