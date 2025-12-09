#include <Wire.h>
#include <AS5600.h>


AS5600 encoder; // Create an AS5600 object
float diff_prev = 0; //pidr

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
  int dt = 10;
  uint16_t rawAngle = encoder.readAngle();
  float rad_angle = radiate(rawAngle);
  Serial.println(rad_angle);
  float diff_value = diff(rad_angle, diff_prev, dt);
  Serial.println(diff_value);
  diff_prev = rad_angle;
  delay(dt);
}
