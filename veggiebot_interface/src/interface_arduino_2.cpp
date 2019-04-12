#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

//tf::TransformBroadcaster tfbroadcaster;
float _latitude, _longitude, _quatx, _quaty, _quatz, _x, _y, _tetha=0;
float _quatw =1;
float _vEncRight, _vEncLeft, _v, _w, _arduinoTime, _mx, _my, _mz=0;

float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

void imuCallback(const std_msgs::Float32MultiArray& msg)
{
	_latitude    = msg.data[0];
	_longitude   = msg.data[1];
	_quatw       = msg.data[2];
	_quatx       = msg.data[3];
	_quaty       = msg.data[4];
	_quatz       = msg.data[5];
    _x           = msg.data[6];
    _y           = msg.data[7];
    _tetha       = msg.data[8];
    _vEncRight   = msg.data[9];
    _vEncLeft    = msg.data[10];
    _v           = msg.data[11];
    _w           = msg.data[12];
    _arduinoTime = msg.data[13];
  //  _mx          = msg.data[14];
  //  _my          = msg.data[15];
  //  _mz          = msg.data[16];
}

int main(int argc, char** argv){
 ros::init(argc, argv, "interfacebot");
 ros::NodeHandle nh2; 

 nav_msgs::Odometry odom; 
 sensor_msgs::Imu imu; 
 sensor_msgs::JointState joint_states; 
 geometry_msgs::TransformStamped imu_tf, odom_tf;
 static tf::TransformBroadcaster tfbroadcaster;

 ros::Publisher imu_pub = nh2.advertise<sensor_msgs::Imu>("imu_data", 50);
 ros::Publisher odom_pub = nh2.advertise<nav_msgs::Odometry>("odom", 50);
 ros::Subscriber imu_sub = nh2.subscribe("float_robot_state",1000,imuCallback);
 ros::Publisher joint_states_pub = nh2.advertise<sensor_msgs::JointState>("joint_states", 50);
 
 joint_states.name.resize(2);
 joint_states.position.resize(2);
 joint_states.velocity.resize(2);
 joint_states.name[0]=("wheel_left_joint");
 joint_states.name[1]=("wheel_right_joint");

 while(ros::ok())
  {
  	geometry_msgs::Quaternion odom_quat =tf::createQuaternionMsgFromYaw(_tetha);
  	ros::Time current_time = ros::Time::now();

	odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
 
    odom.pose.pose.position.x  = _x; //set positions 
    odom.pose.pose.position.y  = _y;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation = odom_quat;
  
    odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
    odom.twist.twist.linear.x  = _v;
    odom.twist.twist.linear.y  =  0;
    odom.twist.twist.angular.z = _w;

    imu.header.stamp = current_time;
	imu.header.frame_id = "base_link";
	imu.orientation.x = _quatx;
	imu.orientation.y = _quaty;
	imu.orientation.z = _quatz;
	imu.orientation.w = _quatw;
	 // // imu.angular_velocity.x = msg.data[4];
	 // // imu.angular_velocity.y = msg.data[5];
	 // // imu.angular_velocity.z = msg.data[6];
	 // // imu.linear_acceleration.x = msg.data[7];
	 // // imu.linear_acceleration.y = msg.data[8];
	 // // imu.linear_acceleration.z = msg.data[9];

	imu_tf.header.stamp    = current_time;
	imu_tf.header.frame_id = "base_link";
	imu_tf.child_frame_id  = "imu_link";
	imu_tf.transform.rotation.w = _quatw;
	imu_tf.transform.rotation.x = _quatx;
	imu_tf.transform.rotation.y = _quaty;
	imu_tf.transform.rotation.z = _quatz;

	imu_tf.transform.translation.x = -0.025;
	imu_tf.transform.translation.y = 0.04;
	imu_tf.transform.translation.z = 0.1725;

	odom_tf.header.stamp    = current_time;
	odom_tf.header.frame_id = "odom";
	odom_tf.child_frame_id  = "base_footprint";
	odom_tf.transform.rotation = odom_quat;

    odom_tf.transform.translation.x = _x;
	odom_tf.transform.translation.y = _y;
	odom_tf.transform.translation.z = 0;

    joint_states.header.frame_id ="base_footprint";    
    // joint_states.name_length     = 2;
    // joint_states.position_length = 2;
    // joint_states.velocity_lenth  = 2;
    // joint_states.effort_length   = 2;
    joint_states.header.stamp = current_time;
    joint_states.position[0] = 0; 
    joint_states.position[1] = 0;
    joint_states.velocity[0] = 0;
    joint_states.velocity[1] = 0;

	tfbroadcaster.sendTransform(imu_tf);
	tfbroadcaster.sendTransform(odom_tf);
  	
  	imu_pub.publish(imu);
  	odom_pub.publish(odom);
  	joint_states_pub.publish(joint_states);
  	//printf("%.6f", _quatw);
    ros::spinOnce();
  }

 return 0;
}
