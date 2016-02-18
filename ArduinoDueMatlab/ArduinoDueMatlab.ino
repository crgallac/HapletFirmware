#include<stdlib.h>
#include <math.h>
#include <Encoder.h>
#include <pwm01.h>


// MOTOR SIGNAL VARIABLES
   uint32_t  pwm_duty0;
   uint32_t pwm_duty1; 
   uint32_t  pwm_freq = 40000; //Hz 
   uint32_t pwmPin1=8; //J2
   uint32_t dirPin1= 26; 
   uint32_t pwmPin0=9; //J3
   uint32_t dirPin0= 22; 
   float duty0; 
   float duty1; 


// COMMAND SIGNAL VARIABLES

char operation; // Holds operation (R, W, ...)
char mode; // Holds the mode (F, T, ...)
int wait_for_transmission = 0; // Delay in us in order to receive the serial data

//TRANSMITTED DATA
union binary_float {
  float floating_point;
  char binary_array[4];
};

union binary_float torques[2];
union binary_float angles[2];

// Global Vars

//angles
long newPosition0  = -999;
long newPosition1  = -999;
float th1;
float th2;
float ts1; 
float ts0; 

//forces
float Tp0;
float Tp1; 

//encoder
   Encoder myEnc0(28,29); //pins for J3
   Encoder myEnc1(24, 25);// pins for J2

// Define kinematic parameters 
//double rh = 0.0725;   //[m]
double l=0.067; 
double L=0.073;
double d=0.020; 


void setup() {
  // Serial Communication
    Serial.begin(115200);
    Serial.setTimeout(100); //ensure serial initialized 
    Serial.println("Haplet:");

   float offset=0.0; 

   myEnc0.write(-(180.0-offset)*13856.0/360.0);
   myEnc1.write(-offset*13856.0/360.0);

pinMode(dirPin0, OUTPUT); 
pinMode(dirPin1,OUTPUT); 

   pwm_set_resolution(12);

   pwm_setup(pwmPin0 , pwm_freq, 1);  // Pin 8 freq set to "pwm_freq2" on clock 
   pwm_setup(pwmPin1, pwm_freq, 1);  // Pin 9 freq set to "pwm_freq2" on clock B



}


void loop() {

//test motor wiring
// pwm_write_duty( pwmPin0, 1000 );  // 50% duty cycle on Pin 8

  // put your main code here, to run repeatedly:

 //current raw position the rotary encoder
  newPosition0 = myEnc0.read(); //J3
  newPosition1 = myEnc1.read();  //J2


  ts1= -360.0/13824.0*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  ts0 = -360.0/13824.0*newPosition0; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  
  th1=ts0*3.14159/180.0; 
  th2=ts1*3.14159/180.0; 

  angles[0].floating_point = th1;
  angles[1].floating_point= th2;


//COMMAND INTERPRETER
 if (Serial.available() > 0) {
    operation = Serial.read();
    delayMicroseconds(wait_for_transmission); // If not delayed, second character is not correctly read
    mode = Serial.read();
   delayMicroseconds(wait_for_transmission); // If not delayed, second character is not correctly read
    if (Serial.read()==':'){

          for(int i=0; i<4; i++){
           torques[0].binary_array[i]= Serial.read();
          }

            for(int i=0; i<4; i++){
           torques[1].binary_array[i]= Serial.read();
          }

    }

    switch (operation){
    case 'R': // Read operation, e.g. RJ, RP
      if (mode == 'P'){ //  read position
//        return_position(x,y,xdot,ydot);  // this is a function
      }
      else if (mode == 'J'){ // Analog read
        return_joints(angles[0].floating_point, angles[1].floating_point);
      }
      else {
        break; // Unexpected mode
      }
      break;

    case 'W': // Write operation, e.g. WD3:1, WA8:255
      if (mode == 'F'){ //  write force
//        write_force(fx, fy);
      }
      else if (mode == 'T'){ //  write torque
//        noInterrupts(); 
        write_torque(torques[0].floating_point, torques[1].floating_point);
//        interrupts(); 
      }
      else {
        break; // Unexpected mode
      }
      break;

    default: // Unexpected char
      break;
    }
}


}

void return_joints(float th1, float th2){

angles[0].floating_point= th1; 
angles[1].floating_point=th2; 
   
Serial.print('J');
     for(int i=0; i<4; i++){
           Serial.print(angles[0].binary_array[i]);
          }
          for(int i=0; i<4; i++){
                    Serial.print(angles[1].binary_array[i]);
 }
}

void write_torque(float tau1, float tau2){

   Tp0=torques[0].floating_point; //J3
   Tp1=torques[1].floating_point;//J2
   Tp0=Tp0;
   Tp1=-Tp1;

  // Determine correct direction for motor torque
  if(Tp0 <= 0) {
    digitalWrite(dirPin0, HIGH);
  }
  else {
    digitalWrite(dirPin0, LOW);
  }

  if(Tp1 <=  0) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }

//%%%FIX
  duty0 = abs(Tp0/0.143);
  duty1 = abs(Tp1/0.143);


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

int dutyOut0=4095*duty0;  
int dutyOut1=4095*duty1;  

 pwm_write_duty( pwmPin0, dutyOut0 );  // 50% duty cycle on Pin 8
   pwm_write_duty( pwmPin1, dutyOut1 );  // 50% duty cycle on Pin 9


return_joints(angles[0].floating_point, angles[1].floating_point);
//  return_joints(angles[0].floating_point, angles[1].floating_point);
}


