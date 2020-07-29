#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <CytronMotorDriver.h>
#include <math.h>

unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
//float         last_y_angle;
//float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
//float         last_gyro_y_angle;
//float         last_gyro_z_angle;

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

//float    base_x_gyro;
float    base_y_gyro;
//float    base_z_gyro;

inline unsigned long get_last_time() {return last_read_time;} //DONT UNDERSTAND THIS
inline float get_last_x_angle() {return last_x_angle;}
//inline float get_last_y_angle() {return last_y_angle;}
//inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
//inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
//inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}


float sensVal;           // for raw sensor values 
float filterVal = 0.001;        // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;              // this holds the last loop value just use a unique variable for every different sensor that needs smoothing

float averageValue(float GyX);
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

int Kp = 50;
int Ki = 0;
int Kd = 0.1;
int motorspeed = 0;

float RADIANS_TO_DEGREES = 180/3.14159;


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

      calibrate_sensors();
}

void loop(){
 
  Input = smooth(getAngle(),filterVal, smoothedVal);                                               //Removed Smoothing, likely done by complementary filter in "getAngle()"
    
    /*
    float angle = 0;
    float angleAvg = 0;
    int count = 0;
    
    for (int thisReading = 0; thisReading < 10; thisReading++){
        angle = getAngle();
        if( angle > 10 ){
          angleAvg = angleAvg + 5;
          count++;
        }else if( angle < -10 ){
          angleAvg = angleAvg - 5;
          count++;
        }else{
          angleAvg = angleAvg + angle;
          count++;
        }
      }
    Input = angleAvg/count;
    */
    
    
    Serial.println(" ");    //Debugging
    //Serial.print("X: "); 
    Serial.print(Input);
   
    balancePID.Compute();                                                  //Supposed to pass this a "double pos", unsure how this works, possibly for Driven Wheel Balance. Calculates "output = Kp * error + integral- Kd * dInput;"
    
    //Serial.print(" ");      //Debugging
    //Serial.print("PIDOutput:");
    //Serial.print(Output);
    //Serial.println("");

    if(Output > motorspeed){
      for (int thisReading = motorspeed; thisReading < Output; thisReading = thisReading + 2){
        motor.setSpeed(thisReading);
        delay(1);
      }
    }else{
      for (int thisReading = motorspeed; thisReading > Output; thisReading = thisReading - 2){
        motor.setSpeed(thisReading);
        delay(1);
      }
    }

    motorspeed = Output;
  
}


int smooth(float data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

int averageValue(int GyX){
    total= total - readings[index];
    readings[index] = GyX;
    total= total + readings[index];     
    index = index + 1;                    
    if (index >= numReadings){              
      index = 0;                          
    }
    average = total / numReadings;  
    Serial.print("Average Value (X): ");Serial.println(average); 
   return average;   
}

float getAngle() 
{
  double dT;

  float angleTemp = 0;

// Read the raw values.
  sixDOF.getRawValues(rawSixDof);

  float acceX = rawSixDof[0];
  float acceY = rawSixDof[1];
  float acceZ = rawSixDof[2];
  float gyroY = rawSixDof[4];

// Get the time of reading for rotation computations
  unsigned long t_now = millis();

// Convert gyro values to degrees/sec
  float FS_SEL = 3;           //Scalling factor that converts raw GRYO data into degrees/s, will likely have to change. 

  float gyro_x = (gyroY - base_x_gyro)/FS_SEL; //DONT UNDERSTAND THIS, Original code has line: "accel_t_gyro_union accel_t_gyro;" Unsure what this does
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_x = atan(-1*acceY/sqrt(pow(acceX,2) + pow(acceZ,2)))*RADIANS_TO_DEGREES; //Unsure if order of accelerations are correct
  float accel_angle_z = 0;

// Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  
  float gyro_angle_x = gyroX*dt + get_last_x_angle();

// Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyroX*dt + get_last_gyro_x_angle();

// Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;

// Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, unfiltered_gyro_angle_x);

return angle_x;

/* Previous methods, commented out
  //if(rawSixDof[2] < 0){
  //  return ((rawSixDof[0]^2)+(rawSixDof[2]^2))^(1/2);
  //}
  //if(rawSixDof[2] > 0){
  //  return -((rawSixDof[0]^2)+(rawSixDof[2]^2))^(1/2);
  //}

  //angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  //angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);

  angleTemp = (_atan2(rawSixDof[0],rawSixDof[2])/10.0);

*/
}

/* This or Math header
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
*/

void set_last_read_angle_data(unsigned long time, float x, float x_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_gyro_x_angle = x_gyro;
}

void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  
  Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  //read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  sixDOF.getRawValues(rawSixDof);

  float acceX = rawSixDof[0];
  float acceY = rawSixDof[1];
  float acceZ = rawSixDof[2];
  float gyroY = rawSixDof[4];
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    sixDOF.getRawValues(rawSixDof);

    x_accel += rawSixDof[0];
    y_accel += rawSixDof[1];
    z_accel += rawSixDof[2];
    y_gyro += rawSixDof[4];
/*
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
*/

    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  //x_gyro /= num_readings;
  y_gyro /= num_readings;
  //z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  //base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  //base_z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}