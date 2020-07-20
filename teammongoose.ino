#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

int sensVal;           // for raw sensor values 
float filterVal = 0.001;        // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;              // this holds the last loop value just use a unique variable for every different sensor that needs smoothing

    //float smoothedVal2;             // this would be the buffer value for another sensor if you needed to smooth two different sensors - not used in this sketch

    //Start PID balance//
int i, j;  

Servo firstESC;
const int MPU=0x68;             // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int goal = 300;   //?? What ius this ??

double Input;
double Output;
double Setpoint = 0;    //Setpoint of 0 Degrees from Verticle

int Kp = 100;
int Ki = 0;
int Kd = 1;

PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  

    //int neutral = 1300;
    //int fullForward = 2600;
    //int fullReverse = 0;

    //End PID Balance//

    //Start Angle integration//
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

void getAngle();
    //End Angle Integration//

void setup() 
{
      //Start Angle integration//
  Wire.begin();                   /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
      //Serial.begin(9600);
      //End Angle Integration//
  
      //Start PID balance//
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(0,255);
  Serial.begin(9600);         
                                // initialize all the readings to 0:
  /*
  Serial.println("Calibration procedure for Mamba ESC.");
  Serial.println("Turn on ESC.");
  Serial.println("Press any key to continue.");
  while (!Serial.available());
  Serial.read();
  
  firstESC.writeMicroseconds(0);
  Serial.println("Starting Calibration.");
  delay(5000);
  firstESC.writeMicroseconds(2600);
  Serial.println("Writing Full Throttle.");
  delay(5000);
  firstESC.writeMicroseconds(0);
  Serial.println("Writing Full Reverse.");
  delay(5000);
  firstESC.writeMicroseconds(1300);
  Serial.println("Writing Neutral.");
  delay(10000);
  Serial.println("Calibration Complete.");


  for (int thisReading = 0; thisReading < numReadings; thisReading++){ 
    readings[thisReading] = 0;
  }
  */ 
      //End PID Balance//
}

void loop(){
  getAngle();                                                           //Updates the Pitch Value ( Total_angle[0] ), since can't pass an array without pointers. 
  Input = Total_angle[0];                                               //Removed Smoothing, likely done by complementary filter in "getAngle()"

      //Serial.print("Average Value (X): ");
      //Serial.println(Input);
  balancePID.Compute();                                                  //Supposed to pass this a "double pos", unsure how this works, possibly for Driven Wheel Balance. Calculates "output = Kp * error + integral- Kd * dInput;"
  if(Input > 0){                                                         //If Angle <0, turn motor to correct, direction/intensity TBD
    value = (Output*5.6);                                                //5.6 is some sort of multiplication factor, unsure, but will have to be changed/tunned
      motorSpeed(value);
  }
  if(Input < 0){
    value = (Output*5.6);
      motorSpeed(value);
  }
      //Serial.println(value);
}

int motorSpeed(int newValue){
      //firstESC.writeMicroseconds(newValue);
      //  if(Serial.available()){ 
      //      value = Serial.parseInt();
      //  }
      
}
/*
int averageValue(int GyX){
    total= total - readings[index];
    readings[index] = GyX;
    total= total + readings[index];     
    index = index + 1;                    
    if (index >= numReadings){              
      index = 0;                          
    }
    average = total / numReadings;  
    //Serial.print("Average Value (X): ");Serial.println(average); 
   return average;   
}

int readGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.println(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
  
//delay(333);
  if(AcY < 0){
    return ((AcX^2)+(AcY^2))^(1/2);
  }
  if(AcY > 0){
    return -((AcX^2)+(AcY^2))^(1/2);
  }
}


int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}
*/

void getAngle() 
{
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
                                  ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read(); 
                                  /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); 
                                  //////////////////PULLING RAW GYRO DATA FROM IMU/////////////////////////
    Gyr_rawX=Wire.read()<<8|Wire.read(); 
    Gyr_rawY=Wire.read()<<8|Wire.read(); 
                                  ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
    Gyro_angle[0] = Gyr_rawX/131.0; 
    Gyro_angle[1] = Gyr_rawY/131.0;
                                  //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
                                  //Serial.print(Total_angle[0]);
                                  //Serial.print("<-----pitch               roll-------->");
                                  //Serial.print(Total_angle[1]);
                                  //Serial.println("");
                                  ////////////////////////////////The angles update slowly beacuse of so many Serial prints///////////
                                  ///////////////////this is just demonstration///////////////////////////
}