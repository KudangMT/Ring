#include <Wire.h>
#include <AS5600.h>

uint8_t v9 = 0;
uint8_t v10 = 0;
uint8_t r = 50;
AS5600 encoder; // Create an AS5600 object
float diff_prev = 0; //pidr
int t = v9;
float radiate (float data){ // absolute-》radian
  float rad = data * (2*PI)/4096;
  return rad;
}

float diff(float x1, float x2, float dt){
  return (x2-x1)/dt*1000;
}

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C
}

  

void loop() {
  
  while(r)
  {
  //   //digitalWrite(9, 1);
  //   PORTB|=4;
  //   delayMicroseconds(v9);
  //   //digitalWrite(9, 0);
  //   PORTB&=0XFB;
  //   delayMicroseconds(256 - v9);
  //   r--;
  // }
  // r = 100;
  //     if (PIND&32){
  //   v9++;    
  //   }
  //   if (PIND&16){
  //     v9--;
  //   }
  // Serial.println(v9);
  // //Serial.print(v9>>1);

   

  
  // int dt = 1;
  /*uint16_t rawAngle = encoder.readAngle();
  float rad_angle = radiate(rawAngle);
  //Serial.println(rad_angle);
  float diff_value = diff(rad_angle, diff_prev, t);
  Serial.print(abs(diff_value));
  Serial.print(' ');

*/

  //diff_prev = rad_angle;
   if (PIND&32){
    v9++;    
   }
   if (PIND&16){
    v9--;
   }
   if (digitalRead(6)){
     v10++;
   }
   if (digitalRead(7)){
     v10--;
   }
  analogWrite(9, v9);
  analogWrite(10, v10); 
  Serial.print(v9);
  Serial.print(' ');
  Serial.print(v10);
  Serial.print('\n');
  t = v9;
  }
}
