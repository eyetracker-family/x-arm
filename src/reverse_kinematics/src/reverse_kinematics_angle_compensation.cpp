#include "../include/reverse_kinematics.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "reverse_kinematics_node"); 
    ros::NodeHandle nh; 

	ros::AsyncSpinner spinner(1);
	spinner.start();


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

		p_knmt = &knmt;			

		p_knmt->newPos[0]=ball_robot_sended.point.x*1000+100;
		p_knmt->newPos[1]=ball_robot_sended.point.y*1000-100;
		p_knmt->newPos[2]=ball_robot_sended.point.z*1000+100;



		Optimal_algorithm(p_knmt);
		p_knmt_step1=p_knmt;

		cout << "angle step 1: "\
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

	geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

	//make up for the error
	cout<<"step 2: compute the error!!!"<<endl;


	tf::TransformListener listener;
	tf::StampedTransform joint6_robot;

    geometry_msgs::Pose target_pose,opti_target_pose;

	int opti_i,opti_j,opti_k;
    double optiLoss = 10000;  //minimum this loss
	opti_i=opti_j=opti_k=0;

	moveit::planning_interface::MoveItErrorCode success;

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::MoveGroupInterface group("x_arm_calib");
	group.setPlanningTime(0.1);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	geometry_msgs::Pose ballPose;
	ballPose.orientation.w=1;
	ballPose.orientation.x=0;
	ballPose.orientation.y=0;
	ballPose.orientation.z=0;
	ballPose.position.x =1;
	ballPose.position.y =1;
	ballPose.position.z =1;

	addBall(planning_scene_interface,"base_link","ball1",ballPose,0.04,0);
	try
	{
		//joint6_robot is the transform from frame /joint6_link to frame /robot_link.
		listener.lookupTransform("robot_link", "joint6_link",ros::Time(0), joint6_robot); 
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				for(int k=0;k<3;k++)
				{
					target_pose.orientation.w = joint6_robot.getRotation().getW();
					target_pose.orientation.x = joint6_robot.getRotation().getX();
					target_pose.orientation.y = joint6_robot.getRotation().getY();
					target_pose.orientation.z = joint6_robot.getRotation().getZ();
					target_pose.position.x = joint6_robot.getOrigin().x()-0.01+i*0.01;
					target_pose.position.y = joint6_robot.getOrigin().y()-0.01+j*0.01;
					target_pose.position.z = joint6_robot.getOrigin().z()-0.01+k*0.01;

					//cout<<"target_pose: "<<target_pose<<endl;
					group.setPoseTarget(target_pose);
					success = group.plan(my_plan);   
					if(success)
					{
					   if(jointLoss(my_plan,i,j,k)<optiLoss)
					   {
					       optiLoss =jointLoss(my_plan,i,j,k);
						   opti_target_pose=target_pose;
						   opti_i=i;opti_j=j;opti_k=k;
					   }
					}
				}
		group.setPoseTarget(opti_target_pose);
		success = group.plan(my_plan);
		cout<<"optimal ijk: "<<opti_i<<" "<<opti_j<<" "<<opti_k<<endl;
		cout<<"angle: "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(5)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(6)<<" "<<endl;
		cout<<"angle error: "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)-p_knmt_step1->inverse_angle.theta1<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)-p_knmt_step1->inverse_angle.theta2<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)-p_knmt_step1->inverse_angle.theta3<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)-p_knmt_step1->inverse_angle.theta4<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)-p_knmt_step1->inverse_angle.theta5<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(5)-p_knmt_step1->inverse_angle.theta6<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(6)-p_knmt_step1->inverse_angle.theta7<<" "<<endl;
	}
	catch (tf::TransformException &ex)
	{
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
	}

	cout<<"input something to confirm the error"<<endl;
	char c;
	cin>>c;

	kinemat_envar_t knmt1;
	kinemat_envar_t *p_knmt1;
	p_knmt1 = &knmt1;
	p_knmt1->newPos[0]=(ball_robot_sended.point.x-0.00)*1000;
	p_knmt1->newPos[1]=(ball_robot_sended.point.y-0.00)*1000;
	p_knmt1->newPos[2]=(ball_robot_sended.point.z+0.00)*1000;

	Optimal_algorithm(p_knmt1);

	cout << "reverse angle step2: "\
		<< p_knmt1->inverse_angle.theta1 << " " << p_knmt1->inverse_angle.theta2 << " "\
		<< p_knmt1->inverse_angle.theta3 << " " << p_knmt1->inverse_angle.theta4 << " "\
		<< p_knmt1->inverse_angle.theta5 << " " << p_knmt1->inverse_angle.theta5 << " "\
		<< p_knmt1->inverse_angle.theta7 << endl;

	cout<<"step 3: make up for the error!!!"<<endl;

	p_knmt1->inverse_angle.theta1=p_knmt1->inverse_angle.theta1-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)-p_knmt_step1->inverse_angle.theta1);
	p_knmt1->inverse_angle.theta2=p_knmt1->inverse_angle.theta2-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)-p_knmt_step1->inverse_angle.theta2);
	p_knmt1->inverse_angle.theta3=p_knmt1->inverse_angle.theta3-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)-p_knmt_step1->inverse_angle.theta3);
	p_knmt1->inverse_angle.theta4=p_knmt1->inverse_angle.theta4-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)-p_knmt_step1->inverse_angle.theta4);
	p_knmt1->inverse_angle.theta5=p_knmt1->inverse_angle.theta5-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)-p_knmt_step1->inverse_angle.theta5);
	p_knmt1->inverse_angle.theta6=p_knmt1->inverse_angle.theta6-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(5)-p_knmt_step1->inverse_angle.theta6);
	p_knmt1->inverse_angle.theta7=p_knmt1->inverse_angle.theta7-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(6)-p_knmt_step1->inverse_angle.theta7);

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
