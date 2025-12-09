//includes, defines
#include <Wire.h>
#include <TimerOne.h>
#include <Servo.h>
#include <AS5600.h>

#define MPU9250_ADDRESS 0x68
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

//I2C logic
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
    {
      // Set register address
      Wire.beginTransmission(Address);
      Wire.write(Register);
      Wire.endTransmission();

      // Read Nbytes
      Wire.requestFrom(Address, Nbytes); 
      uint8_t index=0;
      while (Wire.available())
      Data[index++]=Wire.read();
    }

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
    {
      // Set register address
      Wire.beginTransmission(Address);
      Wire.write(Register);
      Wire.write(Data);
      Wire.endTransmission();
    }
// global initialization

// gyro init
long int ti; // initial time
volatile bool intFlag=false;
// hall's sensor init
AS5600 encoder;
float diff_prev = 0; // global var for angular velocity calculation

// functions init

// time marker, from gyro
void callback()
    { 
      intFlag=true;
      digitalWrite(13, digitalRead(13) ^ 1);
    }

// translating raw input data into radians, from hall's sensor
float radiate (float data){
  float rad = data * (2*PI)/4096;
  return rad;
}
// getting angular velocity from angular position, from hall's sensor
float diff(float x1, float x2, float dt){
  return (x2-x1)/dt*1000;
}

// board setup
  int forward_pressed = 4;
  int backward_pressed = 5;
  int right_pressed = 6;
  int left_pressed = 7;
  int left_leg_level = 30;
  int right_leg_level = 30; // starting levels
  int leg_max = 150;
  int leg_min = 30;
  Servo servo1;
  Servo servo2;
void setup() {
// radio module
// Serial.begin(9600);
  // replace the numbers with the right nano's digital pins for radio module
  pinMode(forward_pressed, INPUT);
  pinMode(backward_pressed, INPUT);
  pinMode(right_pressed, INPUT);
  pinMode(left_pressed, INPUT);

// motor drivers
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);


// hall's sensors

  // Serial.begin(9600);
  Wire.begin(); 
// here we need to somehow connect to different Wires,
// as we have 2 sensors in here and also a gyroscope, all three for i2c
// solve asap


// gyroscope (and accelerometer just in case)
  Wire.begin(); // copper
  Serial.begin(115200);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);

  pinMode(13, OUTPUT);
  Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt

  // Store initial time
  ti=millis();

  servo1.attach(2);
  servo2.attach(3);
} 

// this is where the fun begins
// main loop
void loop(){
  int dt = 1; // period between measurements
  // gyro part, need to rewrite some functions here a little bit
      while (!intFlag);
      intFlag=false;

      // Display time
    //  Serial.print (millis()-ti,DEC);
    //  Serial.print (" ");

      // _______________
      // ::: Counter :::

      // Display data counter
      // Serial.print (cpt++,DEC);
      // Serial.print ("t");

      // ____________________________________
      // ::: accelerometer and gyroscope :::

      // Read accelerometer and gyroscope
      uint8_t Buf[14];
      I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

      // Create 16 bits values from 8 bits data

      // Accelerometer
      int16_t ax=-(Buf[0]<<8 | Buf[1]);
      int16_t ay=-(Buf[2]<<8 | Buf[3]);
      int16_t az=Buf[4]<<8 | Buf[5];

      // Gyroscope
      int16_t gx=-(Buf[8]<<8 | Buf[9]);
      int16_t gy=-(Buf[10]<<8 | Buf[11]);
      int16_t gz=Buf[12]<<8 | Buf[13];

      // Display values
/* // console debug my belowed
     // Accelerometer
     Serial.print (ax,DEC); 
     Serial.print (" ");
     Serial.print (ay,DEC);
     Serial.print (" ");
     Serial.print (az,DEC); 
     Serial.print (" ");

     // Gyroscope
     Serial.print (gx,DEC); 
     Serial.print (" ");
     Serial.print (gy,DEC);
     Serial.print (" ");
     Serial.print (gz,DEC); 
     Serial.print (" ");

     // End of line
     Serial.println("");
     // delay(dt); 
   // end of console debug, my belowed
*/
// hall's part
  uint16_t rawAngle = encoder.readAngle();
  float rad_angle = radiate(rawAngle);
  Serial.println(rad_angle);
  float diff_value = diff(rad_angle, diff_prev, dt);
  Serial.println(diff_value);
  diff_prev = rad_angle;
 // delay(dt);

// reading radio module
  /*int signal_forward = digitalRead(forward_pressed);
  int signal_backward = digitalRead(backward_pressed);
  int signal_right = digitalRead(right_pressed);
  int signal_left = digitalRead(left_pressed);*/

// basic movement control
/*внутренним моторам подобать постоянную скорость, чтобы они двигали колесо на одно и то же расстояние за dt
пока нажата кнопка - плаifвно поднимать обороты до максимума
когда отжата - настроить такое движение, чтобы колесо останавливалось ну хотя бы не из-за трения*/
  
  int time_dilation = 64;
  if(PIND&32){
  for(int seq_break = 0; seq_break < 50; seq_break++){

      PORTB|=2;
      PORTB|=4;
      delayMicroseconds(time_dilation);
      PORTB&=0XFD;
      PORTB&=0XFB;
      delayMicroseconds(256 - time_dilation);
  }
  }
    
    
  
  if(!PIND&32){
      PORTB&=0XFD;
      PORTB&=0XFB;
  }
  // if(PIND&16){
    
  // }
  // if(!PIND&16){
    
  // }
   
//servo turn control
  if(left_pressed){
    //if(left_leg_level + dt * 120/1000 < leg_max){
      servo1.write(0);//left_leg_level + dt * 120/1000
    //  left_leg_level += dt*120/1000;
    //}
  }
  // if(!left_pressed){
  //   if(left_leg_level - dt * 120/1000 > leg_min){
  //     servo1.write(0);
  //     left_leg_level -= dt*120/1000;
  // }
  // }


// servo standup :(
/*считаем, что колесо упало на один бок,
находим бок по положению в пространстве,
нижнюю ногу поднять на 180, верхнюю оттопырить на 45
опускать нижнюю, пока не встанет*/

// servo auto control :(
// need to do some basic math with differentials
// in order to make balance automatic
// ...


}
