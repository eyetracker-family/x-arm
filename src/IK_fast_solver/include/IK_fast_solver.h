#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package
#include <opencv2/opencv.hpp>

#include "IK_fast_solver/grasp.h"

using namespace cv;
using namespace std;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[6],calib_angle_data[18];

void Ser_Arm_Initialize()
{
    try 
    { 
        ser_arm.setPort("/dev/ttyUSB0"); 
        ser_arm.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser_arm.setTimeout(to); 
        ser_arm.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port for Arm"); 
        return; 
    } 
    if(ser_arm.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial_Arm Port initialized"); 
    } 
    else 
    { 
        return; 
    } 
}

void Ser_Hand_Initialize()
{
    try 
    { 
        ser_hand.setPort("/dev/ttyUSB0"); 
        ser_hand.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser_hand.setTimeout(to); 
        ser_hand.open();
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port for Hand"); 
        return; 
    } 
    if(ser_arm.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial_Hand Port initialized"); 
    } 
    else 
    { 
        return; 
    } 
}

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	ball_lscene=*msg;
}

void transformPoint(const tf::TransformListener &listener)
{
	try
	{
		listener.transformPoint("robot_link",ball_lscene,ball_robot);//coordinate transform robot_link
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("exception aroused while coordinate transform!!!");
	}
	//cout<<"ball in robot: "<<endl<<ball_robot<<endl;
}
