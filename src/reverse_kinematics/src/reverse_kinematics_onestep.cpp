#include "matrix_multiply.h"
#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

typedef union theta
{
	unsigned char theta[2];
	short angle;
}theta_t;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[5];

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

int main(int argc,char** argv)
{
    ros::init(argc, argv, "reverse_kinematics_node"); 
    ros::NodeHandle nh; 
    ros::Rate loop_rate(50); 

	Ser_Arm_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	joint_angle_data[0]=0x55;joint_angle_data[1]=0x0b;joint_angle_data[2]=0x05;joint_angle_data[3]=0x0e;//ball pos data header

	if(argc==1)//grasp
	{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected

		ball_robot_sended=ball_robot;

		kinemat_envar_t knmt;
		kinemat_envar_t *p_knmt;
		p_knmt = &knmt;

		p_knmt->newPos[0]=ball_robot_sended.point.x*1000;
		p_knmt->newPos[1]=ball_robot_sended.point.y*1000;
		p_knmt->newPos[2]=ball_robot_sended.point.z*1000;

		cout<<"ball in robot sended!!!: "<<endl<<ball_robot_sended<<endl;

		Optimal_algorithm(p_knmt);

		cout << "reverse angle = "\
			<< p_knmt->inverse_angle.theta1 << " " << p_knmt->inverse_angle.theta2 << " "\
			<< p_knmt->inverse_angle.theta3 << " " << p_knmt->inverse_angle.theta4 << " "\
			<< p_knmt->inverse_angle.theta5 << " " << p_knmt->inverse_angle.theta5 << " "\
			<< p_knmt->inverse_angle.theta7 << endl;

		short int theta=(short int)(p_knmt->inverse_angle.theta1*1000);//plus 1000 to reserve three digit after the dot.
		unsigned char* temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
		joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

		theta=(short int)(p_knmt->inverse_angle.theta2*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta3*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta4*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta5*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta6*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta7*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];


		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"data sended to serial: "<<i<<endl;
	}

	else if(argc==2)//reset
	{
		joint_angle_data[1]=0x04;
		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"arm reseted!!!"<<endl;
	}

	int i=0;
	string arm_returned_cmd;

	geometry_msgs::PointStamped tracker2_tracker2,tracker2_robot,last_tracker2_robot;//for determining whether the arm stops
	geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

    while(ros::ok()) 
    {
		tracker2_tracker2.header.frame_id="tracker2_link";
		tracker2_tracker2.header.stamp=ros::Time();
		tracker2_tracker2.point.x=0;
		tracker2_tracker2.point.y=0;
		tracker2_tracker2.point.z=0;

		tf_robot_lscene_listener.transformPoint("robot_link",tracker2_tracker2,tracker2_robot);//coordinate transform robot_link
		i++;
		if(!(i%30))//get data once every 30 times
        {
			if(fabs(tracker2_robot.point.x-last_tracker2_robot.point.x)<0.02&&fabs(tracker2_robot.point.y-last_tracker2_robot.point.y)<0.02&&fabs(tracker2_robot.point.z-last_tracker2_robot.point.z<0.02))
			{//when arm finish it's movement
				cout<<"arm has stopped!!!"<<endl;
				arm_returned_cmd = ser_arm.read(18+1);
				if(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e)//check the header of frame
				{
					unsigned char theta6[2];
					unsigned char theta7[2];
					theta6[0]=arm_returned_cmd[15];theta6[1]=arm_returned_cmd[14];
					theta7[0]=arm_returned_cmd[17];theta7[1]=arm_returned_cmd[16];

					short int *angle6_=reinterpret_cast<short int*>(theta6);
					short int *angle7_=reinterpret_cast<short int*>(theta7);
			
					cout<<"angle6: "<<(float)*angle6_/1000.0<<endl;
					cout<<"angle7: "<<(float)*angle7_/1000.0<<endl;
					float angle6=((float)*angle6_)/1000.0;
					float angle7=((float)*angle7_)/1000.0;

					hand_tracker2.header.frame_id="tracker2_link";
					hand_tracker2.header.stamp=ros::Time();
					hand_tracker2.point.x=0.04*cos(angle6)+0.07*cos(angle7)*sin(angle6);
					hand_tracker2.point.y=0.04*sin(angle6)-0.099-0.07*cos(angle6)*cos(angle7);
					hand_tracker2.point.z=0.052+0.07*sin(angle7);

					tf_robot_lscene_listener.transformPoint("robot_link",hand_tracker2,hand_robot);//coordinate transform robot_link

					cout<<"hand in robot: "<<endl<<hand_robot<<endl;
					cout<<"tracker2 in robot: "<<endl<<tracker2_robot<<endl;
					
					if(fabs(ball_robot_sended.point.x-hand_robot.point.x)<0.05&&fabs(ball_robot_sended.point.y-hand_robot.point.y)<0.05&&fabs(ball_robot_sended.point.z-hand_robot.point.z<0.05))//if the error between pos of ball and pos of hand is small
					{
						//grasp
						cout<<"grasp!!!"<<endl;
					}
					else
					{
						//make up for the error
						cout<<"make up for the error!!!"<<endl;

						

						kinemat_envar_t knmt1;
						kinemat_envar_t *p_knmt1;
						p_knmt1 = &knmt1;
						p_knmt1->newPos[0]=(2*ball_robot_sended.point.x-hand_robot.point.x)*1000;
						p_knmt1->newPos[1]=(2*ball_robot_sended.point.y-hand_robot.point.y)*1000;
						p_knmt1->newPos[2]=(2*ball_robot_sended.point.z-hand_robot.point.z)*1000;

						Optimal_algorithm(p_knmt1);

						cout << "reverse angle = "\
							<< p_knmt1->inverse_angle.theta1 << " " << p_knmt1->inverse_angle.theta2 << " "\
							<< p_knmt1->inverse_angle.theta3 << " " << p_knmt1->inverse_angle.theta4 << " "\
							<< p_knmt1->inverse_angle.theta5 << " " << p_knmt1->inverse_angle.theta5 << " "\
							<< p_knmt1->inverse_angle.theta7 << endl;

						short int theta=(short int)(p_knmt1->inverse_angle.theta1*1000);//plus 1000 to reserve three digit after the dot.
						unsigned char* temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
						joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

						theta=(short int)(p_knmt1->inverse_angle.theta2*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

						theta=(short int)(p_knmt1->inverse_angle.theta3*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

						theta=(short int)(p_knmt1->inverse_angle.theta4*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

						theta=(short int)(p_knmt1->inverse_angle.theta5*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

						theta=(short int)(p_knmt1->inverse_angle.theta6*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

						theta=(short int)(p_knmt1->inverse_angle.theta7*1000);
						temp=(unsigned char *)&theta;
						joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

						size_t i=ser_arm.write(joint_angle_data,18);
						cout<<"data sended to serial: "<<i<<endl;

					}
				}
			}
            i = 0;
            last_tracker2_robot=tracker2_robot;
        }




		/*arm_returned_cmd = ser_arm.read(18+1);
		if(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e)
		{
			printf("arm_returned_cmd[14]: %#x\n",arm_returned_cmd[14]);
			printf("arm_returned_cmd[15]: %#x\n",arm_returned_cmd[15]);
			printf("arm_returned_cmd[16]: %#x\n",arm_returned_cmd[16]);
			printf("arm_returned_cmd[17]: %#x\n",arm_returned_cmd[17]);

			unsigned char theta6[2];
			unsigned char theta7[2];
			theta6[0]=arm_returned_cmd[15];theta6[1]=arm_returned_cmd[14];
			theta7[0]=arm_returned_cmd[17];theta7[1]=arm_returned_cmd[16];

			printf("theta6[0]: %d\n",theta6[0]);
			printf("theta6[1]: %d\n",theta6[1]);


			short int *angle6_=reinterpret_cast<short int*>(theta6);
			short int *angle7_=reinterpret_cast<short int*>(theta7);
			
			cout<<"angle6: "<<(float)*angle6_/1000.0<<endl;
			cout<<"angle7: "<<(float)*angle7_/1000.0<<endl;
			float angle6=((float)*angle6_)/1000.0;
			float angle7=((float)*angle7_)/1000.0;

			//geometry_msgs::PointStamped hand_tracker2,hand_robot;
			//geometry_msgs::PointStamped tracker2,robot;

			hand_tracker2.header.frame_id="tracker2_link";
			hand_tracker2.header.stamp=ros::Time();
			hand_tracker2.point.x=0.07*cos(angle7)*sin(angle6);
			hand_tracker2.point.y=-0.099-0.07*cos(angle6)*cos(angle7);
			hand_tracker2.point.z=0.052+0.07*sin(angle7);

			tracker2_tracker2.header.frame_id="tracker2_link";
			tracker2_tracker2.header.stamp=ros::Time();
			tracker2_tracker2.point.x=0;
			tracker2_tracker2.point.y=0;
			tracker2_tracker2.point.z=0;


			tf_robot_lscene_listener.transformPoint("robot_link",hand_tracker2,hand_robot);//coordinate transform robot_link
			tf_robot_lscene_listener.transformPoint("robot_link",tracker2_tracker2,tracker2_robot);//coordinate transform robot_link

			cout<<"hand in robot: "<<endl<<hand_robot<<endl;
			cout<<"tracker2 in robot: "<<endl<<tracker2_robot<<endl;

			short int a=(short int)(hand_robot.point.x*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp1=(unsigned char *)&a;//temp[1] is the higher 8 digits
			hand_pos_data[10]=temp1[1];hand_pos_data[11]=temp1[0];//x

			short int b=(short int)(hand_robot.point.y*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp2=(unsigned char *)&b;//temp[1] is the higher 8 digits
			hand_pos_data[12]=temp2[1];hand_pos_data[13]=temp2[0];//x

			short int c=(short int)(hand_robot.point.z*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp3=(unsigned char *)&c;//temp[1] is the higher 8 digits
			hand_pos_data[14]=temp3[1];hand_pos_data[15]=temp3[0];//x

			//size_t i=ser_arm.write(hand_pos_data,16);
			//cout<<"data sended to serial: "<<i<<endl;
			//ROS_INFO_STREAM("Writing to serial port: " <<a/10<<","<<b/10<<","<<c/10);
		}*/

        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}
