#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <CytronMotorDriver.h>


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
int value = 0;

double Input;
double Output;
double Setpoint = 0;    //Setpoint of 0 Degrees from Verticle

int Kp = 30;
int Ki = 0;
int Kd = 10;

PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  

    //End PID Balance//

    //Start Angle integration//
float angle[2]; // pitch & roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6]; 

float _atan2(int32_t y, int32_t x);

float getAngle();

    //End Angle Integration//

    //Start Motor Driver//
   CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.
   //End Motor Driver//

void setup() 
{
      //Start Angle integration//
      Wire.begin();
      
      delay(5);
       sixDOF.init();                        //begin the IMU
       delay(5);
      //End Angle Integration//
  
      //Start PID balance//
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(-255,255);
  Serial.begin(9600);         
      // initialize all the readings to 0:
  
      //End PID Balance//
}

void loop(){
 
  Input = getAngle();                                               //Removed Smoothing, likely done by complementary filter in "getAngle()"
    
    //Serial.println(" ");    //Debugging
    //Serial.print("X:"); 
    //Serial.print(Input);
   
    balancePID.Compute();                                                  //Supposed to pass this a "double pos", unsure how this works, possibly for Driven Wheel Balance. Calculates "output = Kp * error + integral- Kd * dInput;"
    
    //Serial.print(" ");      //Debugging
    //Serial.print("PIDOutput:");
    //Serial.print(Output);
    //Serial.println("");

    motor.setSpeed(Output);

}

/*
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

float getAngle() 
{
  float angleTemp = 0;
     sixDOF.getRawValues(rawSixDof);
  
  angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  //angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);

  angleTemp = (angle[0]/10.0);
  return angleTemp;
}

float _atan2(int32_t y, int32_t x)   //get the _atan2
{
  float z = (float)y / x;
  float a;
  if ( abs(y) < abs(x) )
  {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0)
    {
    if (y<0) a -= 1800;
    else a += 1800;
    }
  }
  else
  {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
}