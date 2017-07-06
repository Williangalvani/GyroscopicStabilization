// MPU-6050 Accelerometer + Gyro

// Bluetooth module connected to digital pins 2,3
// I2C bus on A4, A5
// Servo on pin 0

#include <Wire.h>
//#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>

#define MPU6050_I2C_ADDRESS 0x68

#define SAMPLING_FREQ  100.0 // sample SAMPLING_FREQ in Hz

// Bluetooth transmitter, used optionally
// SoftwareSerial BTSerial(2, 3); // RX | TX

Servo servo;
Servo esc;
// global angle, gyro derived
double gSensitivity = 65.5; // for 500 deg/s, check data sheet
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
int16_t accX = 0, accY = 0, accZ = 0;
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


double gyrXoffs = -97.00, gyrYoffs = 63.00, gyrZoffs = -18.00;

long start;

void setup()
{      
  int error;
  uint8_t c;
  uint8_t sample_div;

  //BTSerial.begin(38400);
  Serial.begin(115200);

  // debug led
  pinMode(13, OUTPUT); 

  // servo 


  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // PWR_MGMT_1:
  // wake up 
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x6b, 0x00);

  // CONFIG:
  // Low pass filter samples, 1khz sample rate
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x1a, 0x01);

  // GYRO_CONFIG:
  // 500 deg/s, FS_SEL=1
  // This means 65.5 LSBs/deg/s
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);

  // CONFIG:
  // set sample rate
  // sample rate SAMPLING_FREQ = Gyro sample rate / (sample_div + 1)
  // 1kHz / (div + 1) = SAMPLING_FREQ  
  // reg_value = 1khz/SAMPLING_FREQ - 1
  sample_div = 1000 / SAMPLING_FREQ - 1;
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x19, sample_div);

  
//  Serial.write("Calibrating...");
  digitalWrite(13, HIGH);
  //calibrate();
  digitalWrite(13, LOW);
  servo.writeMicroseconds(1500);
//  Serial.write("done.");
  servo.attach(2, 550, 2550);
  esc.attach(11);
  esc.writeMicroseconds(1000);
  delay(4500);
  esc.writeMicroseconds(1700);
  start = millis();

}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void imu()
{
  int error;
  double dT;
  double ax, ay, az;
  unsigned long start_time, end_time;

  start_time = millis();

  read_sensor_data();

  // angles based on accelerometer
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

  // angles based on gyro (deg/s)
  gx = gx + gyrX / SAMPLING_FREQ;
  gy = gy - gyrY / SAMPLING_FREQ;
  gz = gz + gyrZ / SAMPLING_FREQ;

  // complementary filter
  // tau = DT*(A)/(1-A)
  // = 0.48sec
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;

  // check if there is some kind of request 
  // from the other side...


  end_time = millis();

  // remaining time to complete sample time
  delay(((1/SAMPLING_FREQ) * 1000) - (end_time - start_time));
  //Serial.println(end_time - start_time);
}



void calibrate(){

  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6]; 
  int num = 500;
  uint8_t error;

  for (x = 0; x < num; x++){

    error = i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, 6);
    if(error!=0)
    return;

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

  Serial.println("Calibration result:");
  Serial.print(gyrXoffs);
  Serial.print(", ");
  Serial.print(gyrYoffs);
  Serial.print(", ");
  Serial.println(gyrZoffs);
  
} 

void read_sensor_data(){
 uint8_t i2cData[14];
 uint8_t error;
 // read imu data
 error = i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, 14);
 if(error!=0)
 return;

 // assemble 16 bit sensor data
 accX = ((i2cData[0] << 8) | i2cData[1]);
 accY = ((i2cData[2] << 8) | i2cData[3]);
 accZ = ((i2cData[4] << 8) | i2cData[5]);

 gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
 gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
 gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
 
}

// ---- I2C routines

int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);
  if (n != 1)
  return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
  return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
  return (-11);

  return (0);  // return : no error
}


int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);        // write the start address
  if (n != 1)
  return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
  return (error);

  return (0);         // return : no error
}


int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;
  
  error = i2c_write(addr, reg, &data, 1);
  return (error);
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