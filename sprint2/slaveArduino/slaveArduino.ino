#include <Wire.h>
#include <Servo.h>

int I2C_address = 9;
int pin1 = A0;

int pos;
int top = 180;
int middle = 100;
int bottom = 80;

int state = 1;


Servo ball[10];

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_address);
  Wire.onReceive(receiveEvent);

  for (int i = 0; i < 10; i++) {
    ball[i].attach(i);
  }
  sunny();
  
}

void loop() {
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

}

void receiveEvent(int howMany) {
  while(Wire.available()) {
    state = Wire.read();
  }
}

void sunny() {
  int positions[10] = {125,125,180,170,165,180,180,180,160,180};
  for (int x = 0; x < 10; x++) {
    setPos(x, positions[x]); 
  }
}

void rainy() {
  int positions[10] = {100, 30 ,140,40, 90,180,180,180,160,180};
  for (int x = 0; x < 10; x++) {
    setPos(x, positions[x]); 
  }
}

void cloudy() {
  int positions[10] = {50,50,105,95,90,125,170,150,160,125};
  for (int x = 0; x < 10; x++) {
    setPos(x, positions[x]); 
  }
}

void partcloudy() {
  int positions[10] = {125,125,125,115,110,180,180,150,160,150};
  for (int x = 0; x < 10; x++) {
    setPos(x, positions[x]); 
  }
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

