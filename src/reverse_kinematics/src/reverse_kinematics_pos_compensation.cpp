#include "../include/reverse_kinematics.h"

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

	if(argc==1)//grasp two steps method
	{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected

		ball_robot_sended=ball_robot;

		cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;
		cout<<"step1 pos in robot: "<<ball_robot_sended.point.x+0.1<<" "<<ball_robot_sended.point.y-0.1<<" "<<ball_robot_sended.point.z+0.1<<endl;

		kinemat_envar_t knmt;
		kinemat_envar_t *p_knmt;
		p_knmt = &knmt;

		p_knmt->newPos[0]=ball_robot_sended.point.x*1000+100;
		p_knmt->newPos[1]=ball_robot_sended.point.y*1000-100;
		p_knmt->newPos[2]=ball_robot_sended.point.z*1000+100;

		Optimal_algorithm(p_knmt);

		cout << "reverse angle step 1: "\
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

		cout<<"input something to confirm ball's pos"<<endl;
		char c;
		cin>>c;

		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"data sended to serial: "<<i<<endl;
	}

	else if(argc==2)//reset
	{
		joint_angle_data[1]=0x04;
		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"arm reseted!!!"<<endl;
		return 0;
	}
	sleep(5);//sleep 5 seconds to ensure step1 executed.
	cout<<"finish sleeping "<<endl;
	string arm_returned_cmd;
	geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

    while(ros::ok()) 
    {
		cout<<"in the loop waiting for angle6 and angle7"<<endl;
		arm_returned_cmd = ser_arm.read(18+1);
		if(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e)//check the header of frame
		{
			//make up for the error
			cout<<"step 2: make up for the error!!!"<<endl;

			

			unsigned char theta6[2];
			unsigned char theta7[2];
			theta6[0]=arm_returned_cmd[15];theta6[1]=arm_returned_cmd[14];
			theta7[0]=arm_returned_cmd[17];theta7[1]=arm_returned_cmd[16];

			/*printf("arm_returned_cmd[14]: %#x\n",arm_returned_cmd[14]);
			printf("arm_returned_cmd[15]: %#x\n",arm_returned_cmd[15]);
			printf("arm_returned_cmd[16]: %#x\n",arm_returned_cmd[16]);
			printf("arm_returned_cmd[17]: %#x\n",arm_returned_cmd[17]);*/

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

			cout<<"hand  pos in robot: "<<hand_robot.point.x<<" "<<hand_robot.point.y<<" "<<hand_robot.point.z<<endl;
			cout<<"error: "<<ball_robot_sended.point.x+0.1-hand_robot.point.x<<" "<<ball_robot_sended.point.y-0.1-hand_robot.point.y<<" "<<ball_robot_sended.point.z+0.1-hand_robot.point.z<<endl;

			cout<<"input something to confirm hand's pos"<<endl;
			char c;
			cin>>c;

			kinemat_envar_t knmt1;
			kinemat_envar_t *p_knmt1;
			p_knmt1 = &knmt1;
			p_knmt1->newPos[0]=(2*ball_robot_sended.point.x+0.1-hand_robot.point.x-0.00)*1000;
			p_knmt1->newPos[1]=(2*ball_robot_sended.point.y-0.1-hand_robot.point.y-0.02)*1000;
			p_knmt1->newPos[2]=(2*ball_robot_sended.point.z+0.1-hand_robot.point.z+0.02)*1000;

			Optimal_algorithm(p_knmt1);

			cout << "reverse angle step2: "\
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
			break;
		}

        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}
