#include <Servo.h>

Servo servo1;
Servo servo2;
int t;


void setup() {
  servo1.attach(2);
  servo2.attach(3);
  t = 500;
}


void loop() {
  // while(t > -5)
  // {
  //   digitalWrite(2, 1);
  //   delayMicroseconds(15 + t);
  //   digitalWrite(2, 0);
  //   delayMicroseconds(185 - t);
  //   t--;
  // }
  // delay(1000);
  // t = 5;
  
  // servo1.write(90);
  // servo2.write(90);
  // delay(10000);
}
