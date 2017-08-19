#include <Arduino.h>
#include "imu.h"
#include <math.h>
#include <Servo.h>



Servo servo;
Servo esc;

float controle =0; 
float controle_servo = 0;
float controle_anterior=0;

float theta = 0;

double rho = 0;
double drho = 0;
double dtheta = 0;

double obs_rho = 0;
double obs_theta = 0;
double obs_drho = 0;

extern double gx;


long start;

void setup()
{      
  Serial.begin(115200);

  setup_imu();
  
  //  Serial.write("Calibrating...");
  //pinMode(13, OUTPUT); 
  //digitalWrite(13, HIGH);
  //calibrate();
  //digitalWrite(13, LOW);
  

  // Center servo
  servo.attach(2, 550, 2550);
  servo.writeMicroseconds(1500);

  //start Esc
  esc.attach(11);
  esc.writeMicroseconds(1000);
  delay(4500); //wait for boot sequence
  esc.writeMicroseconds(1700); //spin up
  start = millis();
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void atuacao()
{

  controle_servo = mapfloat(theta, -90, 90, 2000, 1000); 
  controle_servo = min(max(controle_servo,800),2200);

  servo.writeMicroseconds(controle_servo);

}


void observador()
{
   double y = rho + drho;

   double dobs_rho =     1.5892*y    -0.0024 * controle   -2.1620 * obs_rho;
   double dobs_drho = - -0.5518*y  +  0.0100 * controle +  2.1037 * obs_drho;
   double dobs_theta =  16.9083*y    -0.4799 * controle  -31.5217 * obs_theta;

   obs_theta+= dobs_theta/SAMPLING_FREQ;
   obs_rho+= dobs_rho/SAMPLING_FREQ;
   obs_drho+= dobs_drho/SAMPLING_FREQ;
}



void controla()
{
  float rho1 = rho;
  rho = gx+5;
  drho = (rho - rho1)*SAMPLING_FREQ;
  dtheta = (theta+ (theta+controle))/(2*SAMPLING_FREQ);
  theta +=  dtheta;
  theta = min(90,max(-90,theta));
  

  controle = -34 * rho - 18 * drho + 3.1 * theta ; 
  //ontrole = -44 * rho - 24 * drho + 3.7 * theta ; 
  //controle = -8.9 * obs_rho - 7.7 * obs_drho + 0.78 * obs_theta ; 
  



}


void loop(){
  imu();
  Serial.print(theta);
  Serial.print(",");  
  Serial.println(obs_theta);
  //Serial.print(",");
  //Serial.print(drho);
  //Serial.print(",");
  //Serial.println(controle);

  if (millis()-start > 10000)
  {
    controla();
    observador();
    atuacao();
  }
}