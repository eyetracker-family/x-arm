#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

#include "../include/addObject.h"
#include "../include/collisionCheck.h"

#include "reverse_kinematics/solutions.h"
#include "reverse_kinematics/solution.h"
#include "reverse_kinematics/querySolutions.h"

using namespace std;

typedef union theta
{
	unsigned char theta[2];
	short angle;
}theta_t;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[6],calib_angle_data[18];

double jointLoss(moveit::planning_interface::MoveGroupInterface::Plan& plan,int i,int j,int k)
{
	double loss=0;
	loss+=0.33*plan.trajectory_.joint_trajectory.points.back().positions.at(5)*plan.trajectory_.joint_trajectory.points.back().positions.at(5);
	loss+=0.33*plan.trajectory_.joint_trajectory.points.back().positions.at(6)*plan.trajectory_.joint_trajectory.points.back().positions.at(6);
	loss+=sqrt(pow(i*0.01-0.01,2)+pow(j*0.01-0.01,2)+pow(k*0.01-0.01,2));
    return loss;
}

double allAngleJointLoss(moveit::planning_interface::MoveGroupInterface::Plan& plan,tf2::Matrix3x3& targetPoseRotationMatrix)
{
    double angle0Range = 1.57+0.52;
    double angle1Range = 2.61+0.17;
    double angle2Range = 1.57+0.26;
    double angle3Range = 2.27;
    double angle4Range = 3.14;
    double angle5Range = 3.14;
    double angle6Range = 0.52;

    double angle0Center = (1.57+0.52)/2-0.52;
    double angle1Center = (2.61+0.17)/2-0.17;
    double angle2Center = (1.57+0.26)/2-0.26;
    double angle3Center = 2.27/2;
    double angle4Center = 0;
    double angle5Center = 0;
    double angle6Center = 0;
    double loss=0;

    /*loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(0)-angle0Center)/angle0Range);
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(1)-angle1Center)/angle1Range);
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(2)-angle2Center)/angle2Range);
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(3)-angle3Center)/angle3Range);
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(4)-angle4Center)/angle4Range);*/
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(5)-angle5Center)/angle5Range);
    loss+=fabs((plan.trajectory_.joint_trajectory.points.back().positions.at(6)-angle6Center)/angle6Range);
    loss+=targetPoseRotationMatrix.getColumn(2).y();
    

    return loss;
}

double distanceBetweenQuaterions(tf2::Quaternion& targetPoseOritation, tf2::Quaternion& oritationBeforeOffset)
{
    double loss;
    loss = targetPoseOritation.angleShortestPath(oritationBeforeOffset);
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
