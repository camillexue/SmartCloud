
//      ******************************************************************
//      *                                                                *
//      *              Stepper Driver Test - UnipolarStepper             *
//      *                                                                *
//      *                                                                *
//      ******************************************************************


#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "UnipolarStepper.h"


//
// pin assignments
//
const int MOTOR1_IN1 = 13;
const int MOTOR1_IN2 = 12;
const int MOTOR1_IN3 = 11;
const int MOTOR1_IN4 = 10;

const int MOTOR2_IN1 = 9;
const int MOTOR2_IN2 = 8;
const int MOTOR2_IN3 = 7;
const int MOTOR2_IN4 = 6;

const int MOTOR3_IN1 = 5;
const int MOTOR3_IN2 = 4;
const int MOTOR3_IN3 = 3;
const int MOTOR3_IN4 = 2;

const int MOTOR4_IN1 = A0;
const int MOTOR4_IN2 = A1;
const int MOTOR4_IN3 = A2;
const int MOTOR4_IN4 = A3;

const int FULL_STEPS_PER_REVOLUTION = 2048;



//
// create the stepper motor object
//
UnipolarStepper stepper1;
UnipolarStepper stepper2;
UnipolarStepper stepper3;
UnipolarStepper stepper4;



// ---------------------------------------------------------------------------------
//                              Hardware and software setup
// ---------------------------------------------------------------------------------

//
// top level setup function
//
void setup()
{ 

  stepper1.connectToPins(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4);
  stepper2.connectToPins(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4);
  stepper3.connectToPins(MOTOR3_IN1, MOTOR3_IN2, MOTOR3_IN3, MOTOR3_IN4);
  stepper4.connectToPins(MOTOR4_IN1, MOTOR4_IN2, MOTOR4_IN3, MOTOR4_IN4);

}

void loop()
{
  //
  // set the number of steps per revolutions, 4096 is typical for a 28BYJ-48
  //
  stepper1.setStepsPerRevolution(4096);
  stepper2.setStepsPerRevolution(4096);
  stepper3.setStepsPerRevolution(4096);
  stepper4.setStepsPerRevolution(4096);
  
  //
  // set the speed in rotations/second and acceleration in rotations/second/second
  //
  stepper1.setSpeedInRevolutionsPerSecond(.4);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper2.setSpeedInRevolutionsPerSecond(.4);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper3.setSpeedInRevolutionsPerSecond(.4);
  stepper3.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  stepper4.setSpeedInRevolutionsPerSecond(.4);
  stepper4.setAccelerationInRevolutionsPerSecondPerSecond(.2);
  
  
  //
  // setup motor 1 to move forward 1.0 revolutions, this step does not actually move the motor
  //
  stepper1.setupMoveInRevolutions(4.0);
  stepper2.setupMoveInRevolutions(4.0);
  stepper3.setupMoveInRevolutions(4.0);
  stepper4.setupMoveInRevolutions(4.0);
  
  //
  // execute the moves
  //
  while((!stepper1.motionComplete()) || (!stepper2.motionComplete()) || (!stepper3.motionComplete()) || (!stepper4.motionComplete()))
  {
    stepper1.processMovement();
    stepper2.processMovement();
    stepper3.processMovement();
    stepper4.processMovement();
  }

  while(true)   // wait forever so program only runs once
    ;
}


