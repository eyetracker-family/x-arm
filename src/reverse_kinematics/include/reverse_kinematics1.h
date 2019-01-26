#include "matrix_multiply.h"
#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

#include "../include/addObject.h"
#include "../include/collisionCheck.h"


typedef union theta
{
	unsigned char theta[2];
	short angle;
}theta_t;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[5];

kinemat_envar_t knmt;
kinemat_envar_t *p_knmt,*p_knmt_step1;

double jointLoss(moveit::planning_interface::MoveGroupInterface::Plan& plan,int i,int j,int k)
{
	double loss=0;
	loss+=0.33*plan.trajectory_.joint_trajectory.points.back().positions.at(5)*plan.trajectory_.joint_trajectory.points.back().positions.at(5);
	loss+=0.33*plan.trajectory_.joint_trajectory.points.back().positions.at(6)*plan.trajectory_.joint_trajectory.points.back().positions.at(6);
	loss+=sqrt(pow(i*0.01-0.01,2)+pow(j*0.01-0.01,2)+pow(k*0.01-0.01,2));
    return loss;
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
