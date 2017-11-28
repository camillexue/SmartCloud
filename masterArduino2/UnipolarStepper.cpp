
//      ******************************************************************
//      *                                                                *
//      *                  Unipolar Stepper Motor Driver                 *
//	    *  A modified version for the 28BYJ-48 motor and ULN2003 driver	 *
//      *                                                                *
//      *            Stan Reifel                     12/8/2014           *
//      *               Copyright (c) S. Reifel & Co, 2014               *
//      *                                                                *
//      ******************************************************************

//
// This driver is used to control one or more unipolar 28BYJ-48 stepper motors and   
// requires ULN2003 driver chip.  The motors are accelerated and decelerated as 
// they travel to the final position.
//
// A limitation of this driver is that once a motion starts, you can NOT change the 
// target position, speed or rate of acceleration until the motion has completed.  
// The only exception to this is that you can issue a "Stop" at any point in time,
// which will cause the motor to decelerate until stopped.
//
// Because this library doesn't allow making changes while moving, it can generate
// a faster step rate than a driver that support changing the target position or
// speed while in motion.
//
// This library can generate a maximum of about 12,500 steps per second using an 
// Arduino Uno.  For example running two motors will reduce the step rate by half 
// or more.
//
// This stepper motor driver is based on Aryeh Elderman's paper "Real Time Stepper  
// Motor Linear Ramping Just By Addition and Multiplication".  See: 
//                          www.hwml.com/LeibRamp.pdf
//
// It has advantages and disadvantages over David Austin's method.  The advantage 
// is that it is faster, meaning you can generate more steps/second.  The 
// disadvantageis that the speed ramping while accelerating and decelerating is 
// less linear.  This is likely to only be a problem when coordinating multiple 
// axis that all need to start and finish motions precisely at the same time.
//
// Usage:
//    Near the top of the program, add:
//        include "UnipolarStepper.h"
//
//    For each stepper, delcare a global object outside of all functions as follows:
//        UnipolarStepper stepper1;
//        UnipolarStepper stepper2;
//
//    In Setup(), assign stepper pin numbers:
//        stepper1.connectToPins(10, 11);  <<<< WORK ON THIS
//        stepper2.connectToPins(12, 14);
//
//    Move one motor in units of steps:
//        //
//        // set the speed in steps/second and acceleration in steps/second/second
//        //
//        stepper1.setSpeedInStepsPerSecond(100);
//        stepper1.setAccelerationInStepsPerSecondPerSecond(100);
//
//        //
//        // move 200 steps in the backward direction
//        //
//        stepper1.moveRelativeInSteps(-200);
//
//        //
//        // move to an absolute position of 200 steps
//        //
//        stepper1.moveToPositionInSteps(200);
//
//    Move one motor in units of revolutions:
//        //
//        // set the number of steps per revolutions, 4096 is typical for a 28BYJ-48
//        //
//        stepper1.setStepsPerRevolution(4096);
//
//        //
//        // set the speed in rotations/second and acceleration in rotations/second/second
//        //
//        stepper1.setSpeedInRevolutionsPerSecond(1);
//        stepper1.setAccelerationInRevolutionsPerSecondPerSecond(1);
//
//        //
//        // move backward 1.5 revolutions
//        //
//        stepper1.moveRelativeInRevolutions(-1.5);
//
//        //
//        // move to an absolute position of 3.75 revolutions
//        //
//        stepper1.moveToPositionInRevolutions(3.75);
//
//    Move one motor in units of millimeters:
//        //
//        // set the number of steps per millimeter
//        //
//        stepper1.setStepsPerMillimeter(2123);
//
//        //
//        // set the speed in millimeters/second and acceleration in millimeters/second/second
//        //
//        stepper1.setSpeedInMillimetersPerSecond(20);
//        stepper1.setAccelerationInMillimetersPerSecondPerSecond(20);
//
//        //
//        // move backward 15.5 millimeters
//        //
//        stepper1.moveRelativeInMillimeters(-15.5);
//
//        //
//        // move to an absolute position of 125 millimeters
//        //
//        stepper1.moveToPositionInMillimeters(125);
//
//    Move two motors in units of revolutions:
//        //
//        // set the number of steps per revolutions, 4096 is typical for a 28BYJ-48
//        //
//        stepper1.setStepsPerRevolution(4096);
//        stepper2.setStepsPerRevolution(4096);
//
//        //
//        // set the speed in rotations/second and acceleration in rotations/second/second
//        //
//        stepper1.setSpeedInRevolutionsPerSecond(1);
//        stepper1.setAccelerationInRevolutionsPerSecondPerSecond(1);
//        stepper2.setSpeedInRevolutionsPerSecond(1);
//        stepper2.setAccelerationInRevolutionsPerSecondPerSecond(1);
//
//        //
//        // setup motor 1 to move backward 1.5 revolutions, this step does not actually move the motor
//        //
//        stepper1.setupRelativeMoveInRevolutions(-1.5);
//
//        //
//        // setup motor 2 to move forward 3.0 revolutions, this step does not actually move the motor
//        //
//        stepper2.setupRelativeMoveInRevolutions(3.0);
//
//        //
//        // execute the moves
//        //
//        while((!stepper1.motionComplete()) || (!stepper2.motionComplete()))
//        {
//          stepper1.processMovement();
//          stepper2.processMovement();
//        }
//

#include "UnipolarStepper.h"

// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------

//
// constructor for the stepper class
//
UnipolarStepper::UnipolarStepper()
{
  //
  // initialize constants
  //
  in1Pin = 0;
  in2Pin = 0;
  in3Pin = 0;
  in4Pin = 0;
  stepsPerRevolution = 4096.0;
  stepsPerMillimeter = 2048.0;
  currentPosition_InSteps = 0;
  desiredSpeed_InStepsPerSecond = 4096.0/8.0;
  acceleration_InStepsPerSecondPerSecond = 4096.0/8.0;
  currentStepPeriod_InUS = 0.0;
  stepPhase = 0;
  fullOrHalfStepMode = 2;
}



//
// connect the stepper object to the IO pins
//  Enter:  in1PinNumber = IO pin number for motor wire 1 (blue)
//          in2PinNumber = IO pin number for motor wire 2 (pink)
//          in3PinNumber = IO pin number for motor wire 3 (yellow)
//          in4PinNumber = IO pin number for motor wire 4 (orange)
//
void UnipolarStepper::connectToPins(byte in1PinNumber, byte in2PinNumber, byte in3PinNumber, byte in4PinNumber)
{
  //
  // remember the pin numbers
  //
  in1Pin = in1PinNumber;
  in2Pin = in2PinNumber;
  in3Pin = in3PinNumber;
  in4Pin = in4PinNumber;
  
  //
  // configure the IO bits
  //
  pinMode(in1Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);

  pinMode(in2Pin, OUTPUT);
  digitalWrite(in2Pin, LOW);

  pinMode(in3Pin, OUTPUT);
  digitalWrite(in3Pin, LOW);

  pinMode(in4Pin, OUTPUT);
  digitalWrite(in4Pin, LOW);
}



//
// set stepping mode, full steps or half steps
//  Enter:  stepMode = 1 for full steps, 2 for half steps
//
void UnipolarStepper::setStepMode(byte stepMode)
{
  fullOrHalfStepMode = stepMode; 
}


// ---------------------------------------------------------------------------------
//                     Public functions with units in millimeters 
// ---------------------------------------------------------------------------------


//
// set the number of steps the motor has per millimeter
//
void UnipolarStepper::setStepsPerMillimeter(float motorStepPerMillimeter)
{
  stepsPerMillimeter = motorStepPerMillimeter;
}



//
// get the current position of the motor in millimeter, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in millimeter returned
//
float UnipolarStepper::getCurrentPositionInMillimeters()
{
  return((float)currentPosition_InSteps / stepsPerMillimeter);
}



//
// set the current position of the motor in millimeter, this does not move the motor
//
void UnipolarStepper::setCurrentPositionInMillimeters(float currentPositionInMillimeter)
{
  currentPosition_InSteps = (long) round(currentPositionInMillimeter * stepsPerMillimeter);
}



//
// set the maximum speed, units in millimeters/second, this is the maximum speed reached 
// while accelerating
// Note: this can only be called when the motor is stopped
//  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in millimeters/second
//
void UnipolarStepper::setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInMillimetersPerSecond * stepsPerMillimeter;
}



//
// set the rate of acceleration, units in millimeters/second/second
// Note: this can only be called when the motor is stopped
//  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration, units in 
//          millimeters/second/second
//
void UnipolarStepper::setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond)
{
    acceleration_InStepsPerSecondPerSecond = accelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter;
}



//
// home the motor by moving until the homing sensor is activated, then set the position to zero, 
// with units in millimeters
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in a negative directions 
//          speedInMillimetersPerSecond = speed to accelerate up to while moving toward home, units in millimeters/second
//          maxDistanceToMoveInMillimeters = unsigned maximum distance to move toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool UnipolarStepper::moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, 
  long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)
{
  return(moveToHomeInSteps(directionTowardHome, 
                          speedInMillimetersPerSecond * stepsPerMillimeter, 
                          maxDistanceToMoveInMillimeters * stepsPerMillimeter, 
                          homeLimitSwitchPin));
}



//
// move relative to the current position, units are in millimeters, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the  
//          current position in millimeters
//
void UnipolarStepper::moveRelativeInMillimeters(float distanceToMoveInMillimeters)
{
  setupRelativeMoveInMillimeters(distanceToMoveInMillimeters);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in millimeters, no motion  
// occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the current 
//          position in millimeters
//
void UnipolarStepper::setupRelativeMoveInMillimeters(float distanceToMoveInMillimeters)
{
  setupRelativeMoveInSteps((long) round(distanceToMoveInMillimeters * stepsPerMillimeter));
}



//
// move to the given absolute position, units are in millimeters, this function does not return
// until the move is complete
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to move to 
//          in units of millimeters
//
void UnipolarStepper::moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters)
{
  setupMoveInMillimeters(absolutePositionToMoveToInMillimeters);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in millimeters, no motion occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to move to in 
//          units of millimeters
//
void UnipolarStepper::setupMoveInMillimeters(float absolutePositionToMoveToInMillimeters)
{
 setupMoveInSteps((long) round(absolutePositionToMoveToInMillimeters * stepsPerMillimeter));
}



//
// Get the current velocity of the motor in millimeters/second.  This functions is 
// updated while it accelerates up and down in speed.  This is not the desired speed, 
// but the speed the motor should be moving at the time the function is called.  This  
// is a signed value and is negative when the motor is moving backwards.
// Note: This speed will be incorrect if the desired velocity is set faster than
// this library can generate steps, or if the load on the motor is too great for
// the amount of torque that it can generate.
//  Exit:  velocity speed in millimeters per second returned, signed
//
float UnipolarStepper::getCurrentVelocityInMillimetersPerSecond()
{
  return(getCurrentVelocityInStepsPerSecond() / stepsPerMillimeter);
}



// ---------------------------------------------------------------------------------
//                     Public functions with units in revolutions 
// ---------------------------------------------------------------------------------


//
// set the number of steps the motor has per revolution
//
void UnipolarStepper::setStepsPerRevolution(float motorStepPerRevolution)
{
  stepsPerRevolution = motorStepPerRevolution;
}



//
// get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float UnipolarStepper::getCurrentPositionInRevolutions()
{
  return((float)currentPosition_InSteps / stepsPerRevolution);
}



//
// set the current position of the motor in revolutions, this does not move the motor
//
void UnipolarStepper::setCurrentPositionInRevolutions(float currentPositionInRevolutions)
{
  currentPosition_InSteps = (long) round(currentPositionInRevolutions * stepsPerRevolution);
}



//
// set the maximum speed, units in revolutions/second, this is the maximum speed reached 
// while accelerating
// Note: this can only be called when the motor is stopped
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in revolutions/second
//
void UnipolarStepper::setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInRevolutionsPerSecond * stepsPerRevolution;
}



//
// set the rate of acceleration, units in revolutions/second/second
// Note: this can only be called when the motor is stopped
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration, units in 
//          revolutions/second/second
//
void UnipolarStepper::setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond)
{
    acceleration_InStepsPerSecondPerSecond = accelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution;
}



//
// home the motor by moving until the homing sensor is activated, then set the position to zero, 
// with units in revolutions
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in a negative directions 
//          speedInRevolutionsPerSecond = speed to accelerate up to while moving toward home, units in revolutions/second
//          maxDistanceToMoveInRevolutions = unsigned maximum distance to move toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool UnipolarStepper::moveToHomeInRevolutions(long directionTowardHome, float speedInRevolutionsPerSecond, 
  long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin)
{
  return(moveToHomeInSteps(directionTowardHome, 
                          speedInRevolutionsPerSecond * stepsPerRevolution, 
                          maxDistanceToMoveInRevolutions * stepsPerRevolution, 
                          homeLimitSwitchPin));
}



//
// move relative to the current position, units are in revolutions, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the  
//          current position in revolutions
//
void UnipolarStepper::moveRelativeInRevolutions(float distanceToMoveInRevolutions)
{
  setupRelativeMoveInRevolutions(distanceToMoveInRevolutions);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in revolutions, no motion  
// occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the current 
//          position in revolutions
//
void UnipolarStepper::setupRelativeMoveInRevolutions(float distanceToMoveInRevolutions)
{
  setupRelativeMoveInSteps((long) round(distanceToMoveInRevolutions * stepsPerRevolution));
}



//
// move to the given absolute position, units are in revolutions, this function does not return
// until the move is complete
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to move to 
//          in units of revolutions
//
void UnipolarStepper::moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions)
{
  setupMoveInRevolutions(absolutePositionToMoveToInRevolutions);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in revolutions, no motion occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to move to in 
//          units of revolutions
//
void UnipolarStepper::setupMoveInRevolutions(float absolutePositionToMoveToInRevolutions)
{
 setupMoveInSteps((long) round(absolutePositionToMoveToInRevolutions * stepsPerRevolution));
}



//
// Get the current velocity of the motor in revolutions/second.  This functions is 
// updated while it accelerates up and down in speed.  This is not the desired speed, 
// but the speed the motor should be moving at the time the function is called.  This  
// is a signed value and is negative when the motor is moving backwards.
// Note: This speed will be incorrect if the desired velocity is set faster than
// this library can generate steps, or if the load on the motor is too great for
// the amount of torque that it can generate.
//  Exit:  velocity speed in revolutions per second returned, signed
//
float UnipolarStepper::getCurrentVelocityInRevolutionsPerSecond()
{
  return(getCurrentVelocityInStepsPerSecond() / stepsPerRevolution);
}



// ---------------------------------------------------------------------------------
//                        Public functions with units in steps 
// ---------------------------------------------------------------------------------


//
// set the current position of the motor in steps, this does not move the motor
// Note: This function should only be called when the motor is stopped
//    Enter:  currentPositionInSteps = the new position of the motor in steps
//
void UnipolarStepper::setCurrentPositionInSteps(long currentPositionInSteps)
{
  currentPosition_InSteps = currentPositionInSteps;
}



//
// get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
long UnipolarStepper::getCurrentPositionInSteps()
{
  return(currentPosition_InSteps);
}



//
// setup a "Stop" to begin the process of decelerating from the current velocity to zero, 
// decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps or revolutions
//
void UnipolarStepper::setupStop()
{
  //
  // move the target position so that the motor will begin deceleration now
  //
  if (direction_Scaler > 0)
    targetPosition_InSteps = currentPosition_InSteps + decelerationDistance_InSteps;
  else
    targetPosition_InSteps = currentPosition_InSteps - decelerationDistance_InSteps;
}



//
// set the maximum speed, units in steps/second, this is the maximum speed reached while 
// accelerating
// Note: this can only be called when the motor is stopped
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void UnipolarStepper::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
}



//
// set the rate of acceleration, units in steps/second/second
// Note: this can only be called when the motor is stopped
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in 
//          steps/second/second
//
void UnipolarStepper::setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond)
{
    acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}



//
// home the motor by moving until the homing sensor is activated, then set the position to zero
// with units in steps
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in a negative directions 
//          speedInStepsPerSecond = speed to accelerate up to while moving toward home, units in steps/second
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool UnipolarStepper::moveToHomeInSteps(long directionTowardHome, float speedInStepsPerSecond, 
  long maxDistanceToMoveInSteps, int homeLimitSwitchPin)
{
  float originalDesiredSpeed_InStepsPerSecond;
  bool limitSwitchFlag;
  
  
  //
  // setup the home switch input pin
  //
  pinMode(homeLimitSwitchPin, INPUT_PULLUP);
  
  
  //
  // remember the current speed setting
  //
  originalDesiredSpeed_InStepsPerSecond = desiredSpeed_InStepsPerSecond; 
 
 
  //
  // if the home switch is not already set, move toward it
  //
  if (digitalRead(homeLimitSwitchPin) == HIGH)
  {
    //
    // move toward the home switch
    //
    setSpeedInStepsPerSecond(speedInStepsPerSecond);
    setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);
    limitSwitchFlag = false;
    while(!processMovement())
    {
      if (digitalRead(homeLimitSwitchPin) == LOW)
      {
        limitSwitchFlag = true;
        break;
      }
    }
    
    //
    // check if switch never detected
    //
    if (limitSwitchFlag == false)
      return(false);
  }


  //
  // the switch has been detected, now move away from the switch
  //
  setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome * -1);
  limitSwitchFlag = false;
  while(!processMovement())
  {
    if (digitalRead(homeLimitSwitchPin) == HIGH)
    {
      limitSwitchFlag = true;
      break;
    }
  }
  
  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return(false);


  //
  // have now moved off the switch, move toward it again but slower
  //
  setSpeedInStepsPerSecond(speedInStepsPerSecond/8);
  setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);
  limitSwitchFlag = false;
  while(!processMovement())
  {
    if (digitalRead(homeLimitSwitchPin) == LOW)
    {
      limitSwitchFlag = true;
      break;
    }
  }
  
  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return(false);


  //
  // successfully homed, set the current position to 0
  //
  setCurrentPositionInSteps(0L);    

  //
  // restore original velocity
  //
  setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
  return(true);
}



//
// move relative to the current position, units are in steps, this function does not return
// until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current position 
//          in steps
//
void UnipolarStepper::moveRelativeInSteps(long distanceToMoveInSteps)
{
  setupRelativeMoveInSteps(distanceToMoveInSteps);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in steps, no motion occurs until 
// processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current position 
//          in steps
//
void UnipolarStepper::setupRelativeMoveInSteps(long distanceToMoveInSteps)
{
  setupMoveInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}



//
// move to the given absolute position, units are in steps, this function does not return
// until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to in units 
//          of steps
//
void UnipolarStepper::moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
  setupMoveInSteps(absolutePositionToMoveToInSteps);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in steps, no motion occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to in 
//          units of steps
//
void UnipolarStepper::setupMoveInSteps(long absolutePositionToMoveToInSteps)
{
  long distanceToTravel_InSteps;
  
  
  //
  // save the target location
  //
  targetPosition_InSteps = absolutePositionToMoveToInSteps;
  

  //
  // determine the period in US of the first step
  //
  ramp_InitialStepPeriod_InUS =  1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
    
    
  //
  // determine the period in US between steps when going at the desired velocity
  //
  desiredStepPeriod_InUS = 1000000.0 / desiredSpeed_InStepsPerSecond;


  //
  // determine the number of steps needed to go from the desired velocity down to a velocity of 0
  // Steps = Velocity^2 / (2 * Accelleration)
  //
  decelerationDistance_InSteps = (long) round((desiredSpeed_InStepsPerSecond * desiredSpeed_InStepsPerSecond) / (2.0 * acceleration_InStepsPerSecondPerSecond));
  
  
  //
  // determine the distance and direction to travel
  //
  distanceToTravel_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTravel_InSteps < 0) 
  {
    distanceToTravel_InSteps = -distanceToTravel_InSteps;
    direction_Scaler = -1;
  }
  else
  {
    direction_Scaler = 1;
  }


  //
  // check if travel distance is too short to accelerate up to the desired velocity
  //
  if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
    decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);


  //
  // start the acceleration ramp at the beginning
  //
  ramp_NextStepPeriod_InUS = ramp_InitialStepPeriod_InUS;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;
  startNewMove = true;
}



//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target position yet
//
bool UnipolarStepper::processMovement(void)
{ 
  unsigned long currentTime_InUS;
  unsigned long periodSinceLastStep_InUS;
  long distanceToTarget_InSteps;

  //
  // check if already at the target position
  //
  if (currentPosition_InSteps == targetPosition_InSteps)
    return(true);

  //
  // check if this is the first call to start this new move
  //
  if (startNewMove)
  {    
    ramp_LastStepTime_InUS = micros();
    startNewMove = false;
  }
    
  //
  // determine how much time has elapsed since the last step (Note 1: this method works  
  // even if the time has wrapped. Note 2: all variables must be unsigned)
  //
  currentTime_InUS = micros();
  periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime_InUS;

  //
  // if it is not time for the next step, return
  //
  if (periodSinceLastStep_InUS < (unsigned long) ramp_NextStepPeriod_InUS)
    return(false);

  //
  // determine the distance from the current position to the target
  //
  distanceToTarget_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTarget_InSteps < 0) 
    distanceToTarget_InSteps = -distanceToTarget_InSteps;

  //
  // test if it is time to start decelerating, if so change from accelerating to decelerating
  //
  if (distanceToTarget_InSteps == decelerationDistance_InSteps)
    acceleration_InStepsPerUSPerUS = -acceleration_InStepsPerUSPerUS;
  
  //
  // execute the step on the rising edge
  //
  if (fullOrHalfStepMode == 2)
    setNextHalfStep(direction_Scaler);
  else
    setNextFullStep(direction_Scaler);
  
  //
  // update the current position and speed
  //
  currentPosition_InSteps += direction_Scaler;
  currentStepPeriod_InUS = ramp_NextStepPeriod_InUS;


  //
  // compute the period for the next step
  // StepPeriodInUS = LastStepPeriodInUS * (1 - AccelerationInStepsPerUSPerUS * LastStepPeriodInUS^2)
  //
  ramp_NextStepPeriod_InUS = ramp_NextStepPeriod_InUS * (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod_InUS * ramp_NextStepPeriod_InUS);
 
 
  //
  // clip the speed so that it does not accelerate beyond the desired velocity
  //
  if (ramp_NextStepPeriod_InUS < desiredStepPeriod_InUS)
    ramp_NextStepPeriod_InUS = desiredStepPeriod_InUS;


  //
  // update the acceleration ramp
  //
  ramp_LastStepTime_InUS = currentTime_InUS;
 
 
  //
  // check if the move has reached its final target position, return true if all done
  //
  if (currentPosition_InSteps == targetPosition_InSteps)
  {
    currentStepPeriod_InUS = 0.0;
    return(true);
  }
    
  return(false);
}



//
// update the IO pins for the next half step
//  Enter:  direction = 1 to step forward, -1 to step backward 
//
void UnipolarStepper::setNextHalfStep(int direction)
{
  //
  // compute the next phase number
  //
  stepPhase += direction;
  
  if (stepPhase <= -1)
    stepPhase = 7;
    
  if (stepPhase >= 8)
    stepPhase = 0;

  //
  // set the coils for this phase
  //
  switch(stepPhase)
  {
    case 0:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, HIGH);
      break; 
    case 1:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, HIGH);
      digitalWrite(in4Pin, HIGH);
      break; 
    case 2:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, HIGH);
      digitalWrite(in4Pin, LOW);
      break; 
    case 3:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, HIGH);
      digitalWrite(in3Pin, HIGH);
      digitalWrite(in4Pin, LOW);
      break; 
    case 4:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, HIGH);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, LOW);
      break; 
    case 5:
      digitalWrite(in1Pin, HIGH); 
      digitalWrite(in2Pin, HIGH);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, LOW);
      break; 
    case 6:
      digitalWrite(in1Pin, HIGH); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, LOW);
      break; 
    case 7:
      digitalWrite(in1Pin, HIGH); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, HIGH);
      break; 
   }
}



//
// update the IO pins for the next full step
//  Enter:  direction = 1 to step forward, -1 to step backward 
//
void UnipolarStepper::setNextFullStep(int direction)
{
  //
  // compute the next phase number
  //
  stepPhase += direction;
  
  if (stepPhase <= -1)
    stepPhase = 3;
    
  if (stepPhase >= 4)
    stepPhase = 0;

  //
  // set the coils for this phase
  //
  switch(stepPhase)
  {
    case 0:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, HIGH);
      digitalWrite(in4Pin, HIGH);
      break; 
    case 1:
      digitalWrite(in1Pin, LOW); 
      digitalWrite(in2Pin, HIGH);
      digitalWrite(in3Pin, HIGH);
      digitalWrite(in4Pin, LOW);
      break; 
    case 2:
      digitalWrite(in1Pin, HIGH); 
      digitalWrite(in2Pin, HIGH);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, LOW);
      break; 
    case 3:
      digitalWrite(in1Pin, HIGH); 
      digitalWrite(in2Pin, LOW);
      digitalWrite(in3Pin, LOW);
      digitalWrite(in4Pin, HIGH);
      break; 
   }
}



//
// Get the current velocity of the motor in steps/second.  This functions is updated
// while it accelerates up and down in speed.  This is not the desired speed, but the 
// speed the motor should be moving at the time the function is called.  This is a 
// signed value and is negative when the motor is moving backwards.
// Note: This speed will be incorrect if the desired velocity is set faster than
// this library can generate steps, or if the load on the motor is too great for
// the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float UnipolarStepper::getCurrentVelocityInStepsPerSecond()
{
  if (currentStepPeriod_InUS == 0.0)
    return(0);
  else
  {
    if (direction_Scaler > 0)
      return(1000000.0 / currentStepPeriod_InUS);
    else
      return(-1000000.0 / currentStepPeriod_InUS);
  }
}



//
// check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool UnipolarStepper::motionComplete()
{
  if (currentPosition_InSteps == targetPosition_InSteps)
    return(true);
  else
    return(false);
}

// -------------------------------------- End --------------------------------------

