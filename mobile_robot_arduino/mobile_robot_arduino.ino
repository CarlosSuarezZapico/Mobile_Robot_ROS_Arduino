
#include "Wire.h"
#include "Arduino.h"
// GPS sensor 
#include <PID_v1.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//====================================================================================
#include <ros.h>
#include <ros/time.h>
#include "Arduino.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
//====================================================================================

//=================================================================================

#define LEFT  0
#define RIGHT 1

#define CONTROL_MOTOR_SPEED_PERIOD       10  //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 20 //hz
#define GPS_DATA 0.25 //hz
//====================================================================================
//-------------VARIABLES FOR THE LOW LEVEL MOTION PLANNING------------------------
//FRONT BUMPERS
#define  PRESS 0
#define NOT_PRESS 1

// VEGGIEBOT MOTOR PWM SIGNAL 7, 6
// WATCHDOG MOTOR PWM SIGNAL  5, 6
int MotorR = 6; //5;
int MotorL = 7;

//VEGGIEBOT LED LIGHTS 9, 8
int RedLED = 9;
int GreenLED = 8;// LAST TIME IT DID NOT WORK

//VEGGIEBOT ENCODERS 2, 3, 18, 19
//WATCHDOG ENCODERS 2, 3, 18, 19
// Functions to manage encoders pulses with interrupts
Encoder leftEnc(18, 19);
Encoder rightEnc(2, 3);

//VEGGIEBOT PUSHBUTTON AND LED PUSHBUTTON 22, 23
//WATCHDOG PUSHBUTTON AND LED PUSHBUTTON 22, 23
int LedState = 0;
//====================================================================================
//================================================================================
// Time registers
unsigned long currentMillis, previousMillis;
//================================================================================
//-------------VARIABLES FOR THE TELEOPERATION MODE ------------------------------
float AngMap, LinMap = 0;
int stop_robot = 0;
unsigned long previousMillisTeleop = 0;
unsigned long currentMillisTeleop = 0;
float counterTeleop = 0;
float LinSum, AngSum = 0;
//================================================================================
//-------------MOTOR CONTROL VARIABLES--------------------------------------------
long leftOldPosition, leftNewPosition, rightOldPosition, rightNewPosition  = 0;
double leftVelocity, rightVelocity = 0;
unsigned long t1, t2, previousSample;
int update_d = 1;
double difLeftPulses, difRightPulses, dift;
double SetLeftVelocity, OutputLeftMotor, SetRightVelocity, OutputRightMotor;
//Specify the links and initial tuning parameters
double Kp = 0.005, Ki = 0.035, Kd = 0.0005;
int samplingTime = 100;
double vLeft, vRight = 0;
int vEncLeft, vEncRight = 0;
double prev_vLeft, prev_vRight = 0;
PID leftPID(&vLeft, &OutputLeftMotor, &SetLeftVelocity, Kp, Ki, Kd, DIRECT);
PID rightPID(&vRight, &OutputRightMotor, &SetRightVelocity, Kp, Ki, Kd, DIRECT);
float LinearVelocity, AngularVelocity = 0;
float LinearTeleop, AngularTeleop = 0;
float LinearTeleop2, AngularTeleop2 = 0;
const float diameterWheel = 0.135; //diameter in meters
const float radiusWheel=0.0675;
const float wheelDistance = 0.275; // distance between both wheels in meters
const float TICK2RAD = 0.00236922;
//Warning: Bigger than 127.5 going backward
//         Smaller than 127.5 going forward
float pwmR = 127.5;
float pwmL = 127.5;
//================================================================================
//================================================================================
//------------- VARIABLES FOR ODOMETRY--------------------------------------------
bool init_encoder_[2]= {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double goal_linear_velocity = 0.0;
double goal_angular_velocity = 0.0;
//================================================================================
int ROSCommand = 1;
int currentState=1;
//-------------SUBSCRIBER ROS FUNCTIONS-------------------------------------------
void messageTwist(const geometry_msgs::Twist& cmd_vel_msg)
{
  LinearVelocity = cmd_vel_msg.linear.x;
  AngularVelocity = cmd_vel_msg.angular.z;
}
int teleopMode;
//================================================================================

//-------------ROS VARIABLES------------------------------------------------------
ros::NodeHandle arNode;
//-------------ROS SUBSCRIBERS----------------------------------------------------
// Subscribers definition
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", messageTwist);
//-------------ROS PUBLISHERS-----------------------------------------------------
std_msgs::Float32MultiArray float_robot_state_msg;
ros::Publisher float_robot_state_pub ("float_robot_state", &float_robot_state_msg);
//================================================================================

//================================================================================
//-------------Declaration for SLAM and navigation--------------------------------
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];
//================================================================================
//-------------TIMERS TO CONTROL DIFFERENT PROCESS--------------------------------
static uint32_t tTime[4]= { 0, 0, 0, 0};
int counter = 0;
//====================================================================================


//====================================================================================
//---------IMU DATA------------------------------------------------------------------
struct Quaternion{
  float x;
  float y;
  float z;
  float w;
  };
Quaternion quatRobot, qCalibrated, invquat, qInput;
Adafruit_BNO055 bno = Adafruit_BNO055(55);  
int calibrated =0;
//====================================================================================
//--------------SETUP FUNCTION--------------------------------------------------------
//====================================================================================
void setup()
{      
   arNode.initNode();
   arNode.getHardware()->setBaud(57600);
   arNode.subscribe(cmd_vel_sub);
   arNode.advertise(float_robot_state_pub);
   
   pinMode(MotorR, OUTPUT);
   pinMode(MotorL, OUTPUT);
   pinMode(RedLED, OUTPUT);
   pinMode(GreenLED, OUTPUT);
   // Encoders pins ( 2 for each encoder)
   pinMode(2, INPUT);
   pinMode(3, INPUT);
   pinMode(18, INPUT);
   pinMode(19, INPUT);
  
   leftPID.SetMode(AUTOMATIC);
   leftPID.SetSampleTime(100);   
   rightPID.SetMode(AUTOMATIC);
   rightPID.SetSampleTime(100);  
   //Initial State with motors stopped
   analogWrite( MotorR, 127.5);
   analogWrite( MotorL, 127.5);

   odom_pose[0] = 0.0;
   odom_pose[1] = 0.0;
   odom_pose[2] = 0.0;

   prev_update_time = millis();
  
   SetLeftVelocity=0;
   SetRightVelocity=0;
  
   Wire.begin();
 
   

   bno.begin();
   bno.setExtCrystalUse(true);

   qCalibrated.w=1; qCalibrated.x=0; qCalibrated.y=0; qCalibrated.z=0;
   quatRobot.w = 1; quatRobot.x = 0; quatRobot.y = 0; quatRobot.z = 0;
   invquat.w = 1;   invquat.x = 0;   invquat.y = 0;   invquat.z = 0;
   qInput.w = 1;    qInput.x = 0;    qInput.y = 0;    qInput.z = 0;
   
   float_robot_state_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 14);
   float_robot_state_msg.layout.dim[0].label = "data";
   float_robot_state_msg.layout.dim[0].size = 14;
   float_robot_state_msg.layout.dim[0].stride = 1*14;
   float_robot_state_msg.layout.data_offset = 0;
   float_robot_state_msg.data_length = 14;
   float_robot_state_msg.data = (float *)malloc(sizeof(float)*14);
}

//====================================================================================
//--------------LOOP FUNCTION---------------------------------------------------------
//==================================================================================== 
void loop() 
{
  
   arNode.spinOnce();
 
   //=========================================================================================
  
   vRight = LinearVelocity + (AngularVelocity * wheelDistance) / 2;
   vLeft = LinearVelocity - (AngularVelocity * wheelDistance) / 2;

   SetLeftVelocity = (vLeft * 2652) / ((diameterWheel) * 3.1416);
   SetRightVelocity = (vRight * 2652) / ((diameterWheel) * 3.1416);
   
   // Because of physical connections on VEGGIEBOT AND WATCHDOG, setpoints must change the sign to the opposite
   SetLeftVelocity = -SetLeftVelocity;
   SetRightVelocity = -SetRightVelocity;
  
   //Receive MotorR and MotorL velocity setpoints from motion planning algorithms
   if (SetLeftVelocity >= 0)
     {
      leftPID.SetOutputLimits(127.5, 255);
     }

   if (SetLeftVelocity < 0)
     {
      leftPID.SetOutputLimits(0, 127.5);
     }

   if (SetRightVelocity >= 0)
     {
      rightPID.SetOutputLimits(127.5, 255);
     }

   if (SetRightVelocity < 0)
     {
      rightPID.SetOutputLimits(0, 127.5);
     }

    if (update_d==1)
     {    
       tTime[0] = millis(); 
       update_d=0;
       rightOldPosition = rightEnc.read();
       leftOldPosition =  leftEnc.read();
     }    
     t2=millis();
  
     if ((t2-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
      {    
       update_d=1;
       rightNewPosition = rightEnc.read();
       leftNewPosition = leftEnc.read();
          
       difRightPulses= rightNewPosition-rightOldPosition ;
       dift=(t2-tTime[0]);
       vRight = difRightPulses/dift; //Pulses/second
       vRight = difRightPulses/dift*1000;
       vEncRight = vRight;
      
       difLeftPulses= leftNewPosition-leftOldPosition ;
       vLeft = difLeftPulses/dift; //Pulses/second
       vLeft = difLeftPulses/dift*1000;
       vEncLeft = vLeft;
       
       rightPID.Compute();
       leftPID.Compute();
              
       analogWrite(MotorL, OutputLeftMotor);
       analogWrite(MotorR, OutputRightMotor); 
      }

     if ((millis()-tTime[1]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
       {
        tTime[1] = millis();
        publishSensorStateMsg();
        publishDriveInformation();
        //-----READ IMU DATA----------------------------------------------------------------------   
        // Documentation- BNO055 Interfacing sensor
        // https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/
        // https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf   
        
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
         //-----READ IMU DATA----------------------------------------------------------------------          
         //===============================================================================     
         //FLOAT DATA TO SEND //float_robot_state_msg.data_length = 14;
         float_robot_state_msg.data[0]  = 0;
         float_robot_state_msg.data[1]  = 0;         
         float_robot_state_msg.data[2]  = qCalibrated.w;
         float_robot_state_msg.data[3]  = qCalibrated.x;
         float_robot_state_msg.data[4]  = qCalibrated.y;
         float_robot_state_msg.data[5]  = qCalibrated.z;
         float_robot_state_msg.data[6]  = odom_pose[0]; // x
         float_robot_state_msg.data[7]  = odom_pose[1]; // y
         float_robot_state_msg.data[8]  = odom_pose[2]; // tetha
         float_robot_state_msg.data[9]  = vEncRight;  
         float_robot_state_msg.data[10] = vEncLeft; 
         float_robot_state_msg.data[11] = odom_vel[0]; //v
         float_robot_state_msg.data[12] = odom_vel[2]; //w
         float_robot_state_msg.data[13] = millis();              
                           
         float_robot_state_pub.publish(&float_robot_state_msg);
       }                  
   }
//===============================================================================
//---------------FUNCTIONS--------------------------------------------------------
//================================================================================
//-------------FUNCTION TO CALCULATE SHARP DISTANCE SENSOR------------------------
float calcDistanceSharp (int pin)
{
  // Read analog to digital converter value
  float ADCValue = (float)analogRead(pin);
  double val = 0;
  // Convert in millimeters and return distance
  val = 200.3775040589502
        - 2.2657665648980 * ADCValue
        + 0.0116395328796 * ADCValue * ADCValue
        - 0.0000299194195 * ADCValue * ADCValue * ADCValue
        + 0.0000000374087 * ADCValue * ADCValue * ADCValue * ADCValue
        - 0.0000000000181 * ADCValue * ADCValue * ADCValue * ADCValue * ADCValue;
  return  val;
}
//================================================================================
//-------------FUNCTION TIMER-----------------------------------------------------
unsigned long millisDelay (long interval, unsigned long previousMillis, int* flag)
{
  unsigned long currentMillis = millis();
  if ( (currentMillis - previousMillis) >= interval)
  {
    previousMillis = currentMillis;
    *flag = 1;
  }
  else
  {
    *flag = 0;
  }
  return previousMillis;
}
//================================================================================
//-------------FUNCTION TO CALCULATE ODOMETRY--------------------------------------
bool updateOdometry(double diff_time)
{ 

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = radiusWheel * (wheel_r + wheel_l) / 2.0;
  delta_theta = atan2f(qCalibrated.x* qCalibrated.y + qCalibrated.w * qCalibrated.z,
                       0.5f - qCalibrated.y * qCalibrated.y - qCalibrated.z * qCalibrated.z) - last_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_theta = atan2f(qCalibrated.x* qCalibrated.y + qCalibrated.w * qCalibrated.z,
                      0.5f - qCalibrated.y * qCalibrated.y - qCalibrated.z * qCalibrated.z);

  return true;
}
//================================================================================
//-------------FUNCTION ENCODER STATE -------------------------------------------
void publishSensorStateMsg()
{
 int32_t current_tick;
 
 current_tick = leftEnc.read();

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = -current_tick + last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = rightEnc.read();

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = -current_tick + last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}
//================================================================================
//-------------Mapping float numers------------------------------------------------
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
//================================================================================
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = arNode.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
 
}

