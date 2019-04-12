#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//---------IMU DATA------------------------------------------------------------------
// Definición estructura quaternion
struct Quaternion{
  float x;
  float y;
  float z;
  float w;
  };
//Llamada a función para conectarse al sensor por I2C
Adafruit_BNO055 bno = Adafruit_BNO055(55);  
// Variables usadas durante programa
int calibrated =0;
Quaternion quatRobot, qCalibrated, invquat, qInput;
//====================================================================================
void setup() {
 Wire.begin(); // Pone en funcionamiento el protocolo I2C
 bno.begin();
 bno.setExtCrystalUse(true);
 // Quaterniones usados mas adelante
 qCalibrated.w=1; qCalibrated.x=0; qCalibrated.y=0; qCalibrated.z=0;
 quatRobot.w = 1; quatRobot.x = 0; quatRobot.y = 0; quatRobot.z = 0;
 invquat.w = 1;   invquat.x = 0;   invquat.y = 0;   invquat.z = 0;
 qInput.w = 1;    qInput.x = 0;    qInput.y = 0;    qInput.z = 0;
}

void loop() {
 // Quaternion solicitado al sensor
 imu::Quaternion qIMU = bno.getQuat();
 quatRobot.w= qIMU.w();
 quatRobot.x= qIMU.x();
 quatRobot.y= qIMU.y();
 quatRobot.z= qIMU.z();      
 
 if (calibrated==0)
    {
      /* Get the four calibration values (0..3) */
      /* Any sensor data re+porting 0 should be ignored, */
      /* 3 means 'fully calibrated" */
      uint8_t system, gyro, accel, mag=0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
     if (gyro >= 2)
        {
          qInput.w = quatRobot.w; qInput.x = quatRobot.x; qInput.y = quatRobot.y; qInput.z = quatRobot.z;
          invquat.w =  qInput.w; invquat.x = -qInput.x; invquat.y = -qInput.y; invquat.z = -qInput.z;
          calibrated=1;
        }
     }
 if (calibrated==1)
     {
       qCalibrated.w = invquat.w*quatRobot.w - invquat.x*quatRobot.x - invquat.y*quatRobot.y - invquat.z*quatRobot.z;
       qCalibrated.x = invquat.w*quatRobot.x + invquat.x*quatRobot.w - invquat.y*quatRobot.z + invquat.z*quatRobot.y;
       qCalibrated.y = invquat.w*quatRobot.y + invquat.x*quatRobot.z + invquat.y*quatRobot.w - invquat.z*quatRobot.x;
       qCalibrated.z = invquat.w*quatRobot.z - invquat.x*quatRobot.y + invquat.y*quatRobot.x + invquat.z*quatRobot.w;
     }
}
