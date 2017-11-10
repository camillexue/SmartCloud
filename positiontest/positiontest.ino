#include <Wire.h>
#include <Servo.h>

int pos;
int pinNumber;

int top = 185;
int middle = 100;
int bottom = 80;

int state = 1;


Servo ball[10];
Servo test;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 10; i++) {
    ball[i].attach(i);
    ball[i].write(100);
  }
  
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0 ) {
    if (Serial.readString() == "set") {
      Serial.print("Enter Servo pin number: ");
      pinNumber = getValue();
      Serial.println(pinNumber);
      
      Serial.print("Enter position value: ");
      pos = getValue();
      Serial.println(pos);

      setPos(pinNumber, pos);
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

