#include <Wire.h>
#include <Servo.h>

int I2C_address = 9;
int pin1 = A0;

int pos;
int top = 185;
int middle = 100;
int bottom = 80;

int state = 1;


Servo ball[10];

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_address);
  Wire.onReceive(receiveEvent);

  for (int i = A0; i <= A4; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  for (int i = 0; i < 10; i++) {
    ball[i].attach(i);
    setPos(i, top);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(state) {
    case 1: //sunny
      pos = 175;
      break;
    case 2: //partly cloudy
      pos = 145;
      break;
    case 3: //cloudy
      pos = 110;
      break;
    case 4: //rainy
      pos = 80;
      break;
     default: //nothing
      pos = 185;
      break;
  }

  for (int i = 0; i < 10; i++) {
    setPos(i, pos);
  }
}

void receiveEvent() {
  while(Wire.available()) {
    state = Wire.read();
  }
}

void sunny() {
  for (int i = 0; i < 5; i++) {
      setPos(i, top);
      //ball[i].write(top);
  }
  
}

void rainy() {
  //coordinate ball positions for rainy condition
}

void cloudy() {
  //coordinate ball positions for cloudy condition
}

void partcloudy() {
  //coordinate ball positions for partcloud condition
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
  

