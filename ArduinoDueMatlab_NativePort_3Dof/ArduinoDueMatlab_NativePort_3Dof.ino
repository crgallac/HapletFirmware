#include<stdlib.h>
#include <math.h>
#include <Encoder.h>
#include <pwm01.h>
#include <DueTimer.h>

// MOTOR SIGNAL VARIABLES
   uint32_t  pwm_duty0;
   uint32_t pwm_duty1; 
   uint32_t pwm_duty2; 
   uint32_t  pwm_freq = 40000; //Hz 
     uint32_t pwmPin2=7; //J5 motor 3
   uint32_t dirPin2= 34; 
   uint32_t pwmPin1=8; //J2 motor 2
   uint32_t dirPin1= 22; 
   uint32_t pwmPin0=9; //J3 motor 1
     uint32_t dirPin0= 26; 
   float duty0; 
   float duty1; 
   float duty2; 

// COMMAND SIGNAL VARIABLES

byte COMMAND[16];

byte operation; // Holds operation (R, W, ...)
byte mode; // Holds the mode (F, T, ...)
//int wait_for_transmission = 1000; // Delay in us in order to receive the serial data

//TRANSMITTED DATA
union binary_float {
  float floating_point;
  byte binary_array[4];
};

union binary_float torques[3];
union binary_float angles[3];

// Global Vars

//angles
long newPosition0  = -999;
long newPosition1  = -999;
long newPosition2  = -999;
float th1;
float th2;
float th3;
float ts2;
float ts1;
float ts0;

//forces
float Tp0;
float Tp1;
float Tp2;

//encoder
Encoder myEnc0(28, 29); //pins for J3
Encoder myEnc1(24, 25);// pins for J2
Encoder myEnc2(36, 37);// pins for J5

// Define kinematic parameters
//double rh = 0.0725;   //[m]
//double l = 0.067;
//double L = 0.073;
//double d = 0.020;


void setup() {
  // Serial Communication
  SerialUSB.begin(115200);
  while (!Serial) ;
//  SerialUSB.setTimeout(1); //ensure serial initialized
  //    SerialUSB.println("Haplet Engaged");

  float offset = 0.0;

  myEnc0.write(-(180.0 - offset) * 13856.0 / 360.0);
  myEnc1.write(-offset * 13856.0 / 360.0);
    myEnc2.write(-offset * 13856.0 / 360.0);

  pinMode(dirPin0, OUTPUT);
   pinMode(pwmPin0, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

   pwm_set_resolution(12);

  pwm_setup(pwmPin0 , pwm_freq, 1);  // Pin 8 freq set to "pwm_freq2" on clock
  pwm_setup(pwmPin1, pwm_freq, 1);  // Pin 9 freq set to "pwm_freq2" on clock B
    pwm_setup(pwmPin2, pwm_freq, 1);  // Pin 9 freq set to "pwm_freq2" on clock B


  Timer1.attachInterrupt(write_torque).start(1000);
}

void loop() {
//  while (1) {
    //test motor wiring
    // pwm_write_duty( pwmPin0, 1000 );  // 50% duty cycle on Pin 8

    // put your main code here, to run repeatedly:

    //current raw position the rotary encoder
      newPosition0 =  //J3
      newPosition1 =   //J2
      newPosition2 =   //J5

      ts2= -360.0/13824.0*myEnc2.read(); // Angle of the motor //J5
      ts1= -360.0/13824.0*myEnc1.read(); // Angle of the motor //J2
      ts0 = -360.0/13824.0*myEnc0.read(); //Angle of the motor //J3

       
      angles[0].floating_point = ts0*3.14159/180.0;
      angles[1].floating_point=   ts1*3.14159/180.0;   
      angles[2].floating_point = ts2*3.14159/180.0; 

if(SerialUSB.available()){
SerialUSB.readBytes(COMMAND,4);
get_torques();  
return_joints(); 
SerialUSB.write(COMMAND ,4);  
}
}

void get_torques() {
SerialUSB.readBytes(&(torques[0].binary_array[0]),4); 
SerialUSB.readBytes(&(torques[1].binary_array[0]),4); 
SerialUSB.readBytes(&(torques[2].binary_array[0]),4); 
}

void return_joints() {
SerialUSB.write(&(angles[0].binary_array[0]) ,4);
SerialUSB.write(&(angles[1].binary_array[0]),4); 
SerialUSB.write(&(angles[2].binary_array[0]),4); 
}

void write_torque() {

  Tp0 = torques[0].floating_point; //J3
  Tp1 = -torques[1].floating_point; //J2
   Tp2 = -torques[2].floating_point; //J5

  // Determine correct direction for motor torque
  if (Tp0 <= 0) {
    digitalWrite(dirPin0, HIGH);
  }
  else {
    digitalWrite(dirPin0, LOW);
  }

  if (Tp1 <=  0) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }

// havn't checked this
  if (Tp2 <=  0) {
    digitalWrite(dirPin2, HIGH);
  }
  else {
    digitalWrite(dirPin2, LOW);
  }

  //%%%FIX
  duty0 = abs(Tp0 / 0.143);
  duty1 = abs(Tp1 / 0.143);
  duty2 = abs(Tp2 / 0.143);

  if (duty0 > 1) {
    duty0 = 1;
  }
  else if (duty0 < 0) {
    duty0 = 0;
  }
  if (duty1 > 1) {
    duty1 = 1;
  }
  else if (duty1 < 0) {
    duty1 = 0;
  }
  if (duty2 > 1) {
    duty2 = 1;
  }
  else if (duty2 < 0) {
    duty2 = 0;
  }


  int dutyOut0 = 4095 * duty0;
  int dutyOut1 = 4095 * duty1;
  int dutyOut2 = 4095 * duty2;

pwm_write_duty( pwmPin2, dutyOut2 );  // 50% duty cycle on Pin 7
  pwm_write_duty( pwmPin0, dutyOut0 );  // 50% duty cycle on Pin 8
  pwm_write_duty( pwmPin1, dutyOut1 );  // 50% duty cycle on Pin 9

}


