#include <ros.h>
#include <ros/time.h>
#include "Arduino.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

// variables que recibiran los comandos de velocidad
float LinearVelocity, AngularVelocity = 0;

// función callback para subscriptor de velocidad
void messageTwist(const geometry_msgs::Twist& cmd_vel_msg)
{
  LinearVelocity = cmd_vel_msg.linear.x;
  AngularVelocity = cmd_vel_msg.angular.z;
}

// HANDLERS para utilizar ROS
ros::NodeHandle arNode;
//Handler del subscriptor que recibirá mensajes de tipo geometry_msgs::Twist y
// escuchará por el topic "fixed_cmd_vel"
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("fixed_cmd_vel", messageTwist);
// Handler del publicador que enviará mensaje de tipo std_msgs::Float32MultiArray 
// y los transmitirá a través del topic "float_robot_state"
std_msgs::Float32MultiArray float_robot_state_msg;
ros::Publisher float_robot_state_pub ("float_robot_state", &float_robot_state_msg);


void setup() {
  
   arNode.initNode();
   arNode.getHardware()->setBaud(57600);
   arNode.subscribe(cmd_vel_sub);
   arNode.advertise(float_robot_state_pub);
   
   float_robot_state_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 14);
   float_robot_state_msg.layout.dim[0].label = "data";
   float_robot_state_msg.layout.dim[0].size = 14;
   float_robot_state_msg.layout.dim[0].stride = 1*14;
   float_robot_state_msg.layout.data_offset = 0;
   float_robot_state_msg.data_length = 14;
   float_robot_state_msg.data = (float *)malloc(sizeof(float)*14);
}

void loop() {
  //Revisar que se han recibido nuevos mensajes para el subscriptor
  //Una vez que se reciba un nuevo mensaje las variables LinearVelocity 
  //y AngularVelocity actualizan su valor.
  arNode.spinOnce();

  // Rellenar mensaje publicador para actualizar valores de los sensores
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
  
  // Enviar mensaje por el publicador                      
  float_robot_state_pub.publish(&float_robot_state_msg);
}
