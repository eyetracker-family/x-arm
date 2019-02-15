#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package
#include <opencv2/opencv.hpp>

#include "IK_fast_solver/grasp.h"

using namespace cv;
using namespace std;

short int theta;//plus 1000 to reserve three digit after the dot.
unsigned char* temp;//temp[1] is the higher 8 digits

char c;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[6],calib_angle_data[18];

tf::StampedTransform joint6_robot;
geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

void set_joint_angle(unsigned char* joint_angle_data,IK_fast_solver::grasp grasp_srv)
{
    theta=(short int)(grasp_srv.response.angle0*1000);//plus 1000 to reserve three digit after the dot.
    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
    *(joint_angle_data+4)=temp[1];  *(joint_angle_data+5)=temp[0];

    theta=(short int)(grasp_srv.response.angle1*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+6)=temp[1];  *(joint_angle_data+7)=temp[0];

    theta=(short int)(grasp_srv.response.angle2*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+8)=temp[1];  *(joint_angle_data+9)=temp[0];

    theta=(short int)(grasp_srv.response.angle3*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+10)=temp[1];  *(joint_angle_data+11)=temp[0];

    theta=(short int)(grasp_srv.response.angle4*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+12)=temp[1];  *(joint_angle_data+13)=temp[0];

    theta=(short int)(grasp_srv.response.angle5*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+14)=temp[1];  *(joint_angle_data+15)=temp[0];

    theta=(short int)(grasp_srv.response.angle6*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+16)=temp[1];  *(joint_angle_data+17)=temp[0];
}

void set_joint_angle_const(unsigned char* joint_angle_data,float angle0,float angle1,float angle2,float angle3,float angle4,float angle5,float angle6)
{
    theta=(short int)(angle0*1000);//plus 1000 to reserve three digit after the dot.
    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
    *(joint_angle_data+4)=temp[1];  *(joint_angle_data+5)=temp[0];

    theta=(short int)(angle1*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+6)=temp[1];  *(joint_angle_data+7)=temp[0];

    theta=(short int)(angle2*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+8)=temp[1];  *(joint_angle_data+9)=temp[0];

    theta=(short int)(angle3*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+10)=temp[1];  *(joint_angle_data+11)=temp[0];

    theta=(short int)(angle4*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+12)=temp[1];  *(joint_angle_data+13)=temp[0];

    theta=(short int)(angle5*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+14)=temp[1];  *(joint_angle_data+15)=temp[0];

    theta=(short int)(angle6*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+16)=temp[1];  *(joint_angle_data+17)=temp[0];
}

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
