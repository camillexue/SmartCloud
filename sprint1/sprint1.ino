#include <Servo.h>

Servo ball;
int pos = 185;
int state = 1;

void setup() {
  // put your setup code here, to run once:
  ball.attach(13);

  ball.write(pos);
  delay(1000);
}

void loop() {
  
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
  ball.write(pos);
  
}
