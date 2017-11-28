#include <Wire.h>
#include <Servo.h>

int pos = 0;
int pinNumber = 0;

int top = 185;
int middle = 100;
int bottom = 80;

int state = 1;


Servo ball[10];
Servo test;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 1; i <= 10; i++) {
    ball[i-1].attach(i);
    Serial.println(i);
  }
  int positions1[9] = {125,180,170,165,180,180,180,160,180};
  for (int l = 1; l <= 10; l++) {
    setPos(l, positions1[l-1]); 
  }
  delay(1000);
  Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0 ) {
    if (Serial.readString() == "set") {
      Serial.print("Enter Servo pin number: ");
      pinNumber = getValue() - 1;
      Serial.println(pinNumber);
      
      Serial.print("Enter position value: ");
      pos = getValue();
      Serial.println(pos);
      
      //Serial.print("From: ");
      //Serial.println(ball[pinNumber].read());
      setPos(pinNumber, pos);
      //Serial.print("Moved to: ");
      //Serial.println(ball[pinNumber].read());
    }
  } 
}

//reads serial input, returns int value
int getValue() {
  int value = -1;
  while(value == -1) {
    if (Serial.available() > 0) {
      value = Serial.parseInt();
    }
  }
  return value;
}

void setPos(int pin, int setpos) {
  int currentPos = ball[pin].read();
 
  if (currentPos < setpos) {
    for (int x = currentPos; x <= setpos; x += 1) {
      ball[pin].write(x);
      delay(50);
    }
  }
  if (currentPos > setpos) {
    for (int x = currentPos; x >= setpos; x -= 1) {
      ball[pin].write(x);
      delay(50);
    }
  }
}

