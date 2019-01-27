#include "../include/IK_fast_solver.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "IK_fast_solver_node"); 
    ros::NodeHandle nh; 

    ros::ServiceClient grasp_client = nh.serviceClient<IK_fast_solver::grasp>("grasp");
    IK_fast_solver::grasp grasp_srv;

	Ser_Arm_Initialize();
    //Ser_Hand_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_listener)));

	joint_angle_data[0]=0x55;joint_angle_data[1]=0x0b;joint_angle_data[2]=0x05;joint_angle_data[3]=0x0e;//ball pos data header

	if(argc==1)//reset
	{
		joint_angle_data[1]=0x04;
		size_t i=ser_arm.write(joint_angle_data,18);//reset the arm

        //hand_cmd[0]=0xaa;hand_cmd[1]=0x01;hand_cmd[2]=0x06;hand_cmd[3]=0x00;hand_cmd[4]=0x00;hand_cmd[5]=0x55;
	    //size_t k=ser_hand.write(hand_cmd,6);//reset the hand

		cout<<"arm and hand reseted!!!"<<endl;
		return 0;
	}

    int graspedObjectType=0; // 0:ball, 1:bottle
    graspedObjectType=atoi(argv[1]);

//grasp--two steps method
    if(argc==2)
	{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected


//trajectory planning

        //set_joint_angle_const(joint_angle_data,0.8);

	    theta=(short int)(0.8*1000);//plus 1000 to reserve three digit after the dot.
	    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
	    joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

	    theta=(short int)(0.46*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

	    theta=(short int)(-0.16*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

	    theta=(short int)(1.1*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

	    theta=(short int)(-0.35*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

	    theta=(short int)(0*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

	    theta=(short int)(-0*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

	    ser_arm.write(joint_angle_data,18);
	    cout<<"trajectory planning data sended to serial: "<<endl;




		ball_robot_sended=ball_robot;
        ball_robot_sended.point.x+=0.01;
        //ball_robot_sended.point.z+=0.015;

		cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;//unit : m

// step1 calib
        grasp_srv.request.flag=0;
        grasp_srv.request.x=ball_robot_sended.point.x;
        grasp_srv.request.y=ball_robot_sended.point.y;
        grasp_srv.request.z=ball_robot_sended.point.z;
        grasp_srv.request.q_w=0.5;
        grasp_srv.request.q_x=0.5;
        grasp_srv.request.q_y=-0.5;
        grasp_srv.request.q_z=0.5;
        grasp_client.call(grasp_srv);

		cout << "target angle step 1: "\
			<< grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
			<< grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
			<< grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
			<< grasp_srv.response.angle6 << endl;

		theta=(short int)(grasp_srv.response.angle0*1000);//plus 1000 to reserve three digit after the dot.
		temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
		joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

		theta=(short int)(grasp_srv.response.angle1*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

		theta=(short int)(grasp_srv.response.angle2*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

		theta=(short int)(grasp_srv.response.angle3*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

		theta=(short int)(grasp_srv.response.angle4*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

		theta=(short int)(grasp_srv.response.angle5*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

		theta=(short int)(grasp_srv.response.angle6*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

        for(int i=0;i<18;i++)
            calib_angle_data[i]=joint_angle_data[i];

		cout<<"input something to confirm ball's pos and step1 angle"<<endl;
		cin>>c;

		ser_arm.write(joint_angle_data,18);
		cout<<"data sended to serial: "<<endl;

	    sleep(6);//sleep 5 seconds to ensure step1 executed(the prerequest of caliberation).
	    cout<<"finish sleeping "<<endl;

	    geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

	    //make up for the error
	    cout<<"step 2: compute the error!!!"<<endl;


	    //tf::TransformListener listener;
	    tf::StampedTransform joint6_robot;

        geometry_msgs::Pose target_pose;

	    try
	    {
		    //joint6_robot is the transform from frame /joint6_link to frame /robot_link.
		    tf_listener.lookupTransform("robot_link", "joint6_link",ros::Time(0), joint6_robot); 

            grasp_srv.request.flag=1;
            grasp_srv.request.x=joint6_robot.getOrigin().x();
            grasp_srv.request.y=joint6_robot.getOrigin().y();
            grasp_srv.request.z=joint6_robot.getOrigin().z();
            grasp_srv.request.q_w=joint6_robot.getRotation().getW();
            grasp_srv.request.q_x=joint6_robot.getRotation().getX();
            grasp_srv.request.q_y=joint6_robot.getRotation().getY();
            grasp_srv.request.q_z=joint6_robot.getRotation().getZ();
            grasp_client.call(grasp_srv);

		    //cout << "real angle step 1: "\
			    << grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
			    << grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
			    << grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
			    << grasp_srv.response.angle6 << endl;

		    //cout<<"angle error step1: "<<grasp_srv.response.angle0-grasp_srv.response.angle0<<" "<<grasp_srv.response.angle1-grasp_srv.response.angle1<<" "<<grasp_srv.response.angle2-grasp_srv.response.angle2<<" "<<grasp_srv.response.angle3-grasp_srv.response.angle3<<" "<<grasp_srv.response.angle4-grasp_srv.response.angle4<<" "<<grasp_srv.response.angle5-grasp_srv.response.angle5<<" "<<grasp_srv.response.angle6-grasp_srv.response.angle6<<" "<<endl;
	    }
	    catch (tf::TransformException &ex)
	    {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
	    }

	    cout<<"input something to confirm the error"<<endl;
	    cin>>c;

        grasp_srv.request.flag=2;
        grasp_srv.request.x=ball_robot_sended.point.x;
        grasp_srv.request.y=ball_robot_sended.point.y;
        grasp_srv.request.z=ball_robot_sended.point.z;
        grasp_srv.request.q_w=0;
        grasp_srv.request.q_x=0;
        grasp_srv.request.q_y=0;
        grasp_srv.request.q_z=0;
        grasp_client.call(grasp_srv);

	    cout << "angle step 2: "\
		    << grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
		    << grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
		    << grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
		    << grasp_srv.response.angle6 << endl;

	    cout<<"step 3: make up for the error!!!"<<endl;

	    /*grasp_srv.response.angle0-=(grasp_srv.response.angle0-grasp_srv.response.angle0);
	    grasp_srv.response.angle1-=(grasp_srv.response.angle1-grasp_srv.response.angle1);
	    grasp_srv.response.angle2-=(grasp_srv.response.angle2-grasp_srv.response.angle2);
	    grasp_srv.response.angle3-=(grasp_srv.response.angle3-grasp_srv.response.angle3);
	    grasp_srv.response.angle4-=(grasp_srv.response.angle4-grasp_srv.response.angle4);
	    grasp_srv.response.angle5-=(grasp_srv.response.angle5-grasp_srv.response.angle5);
	    grasp_srv.response.angle6-=(grasp_srv.response.angle6-grasp_srv.response.angle6);*/


	    theta=(short int)(grasp_srv.response.angle0*1000);//plus 1000 to reserve three digit after the dot.
	    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
	    joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

	    theta=(short int)(grasp_srv.response.angle1*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

	    theta=(short int)(grasp_srv.response.angle2*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

	    theta=(short int)(grasp_srv.response.angle3*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

	    theta=(short int)(grasp_srv.response.angle4*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

	    theta=(short int)(grasp_srv.response.angle5*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

	    theta=(short int)(grasp_srv.response.angle6*1000);
	    temp=(unsigned char *)&theta;
	    joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

	    ser_arm.write(joint_angle_data,18);
	    cout<<"data sended to serial: "<<endl;




    //drink water

        sleep(15);

        theta=(short int)(-0.04*1000);//plus 1000 to reserve three digit after the dot.
        temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
        joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

        theta=(short int)(-0.35*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

        theta=(short int)(1.41*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

        theta=(short int)(2*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

        theta=(short int)(0.51*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

        theta=(short int)(0.44*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

        theta=(short int)(-0.35*1000);
        temp=(unsigned char *)&theta;
        joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

        ser_arm.write(joint_angle_data,18);
        cout<<"trajectory planning data sended to serial: "<<endl;
	}


    
    //pull the ball out
   /* sleep(20);
	ser_arm.write(calib_angle_data,18);
	cout<<"ball pulled out,data sended to serial: "<<endl;*/




    //sleep(5);

    /*hand_cmd[0]=0xaa;hand_cmd[1]=0x01;hand_cmd[4]=0x55;
    if(graspedObjectType==0)//ball
    {
        hand_cmd[2]=0x04;hand_cmd[3]=0x03;
    }
    if(graspedObjectType==1)//bottle
    {
        hand_cmd[2]=0x04;hand_cmd[3]=0x03;
    }
	ser_hand.write(hand_cmd,5);//grasp

    sleep(5);
    hand_cmd[1]=0x01;hand_cmd[2]=0x06;hand_cmd[3]=0x00;hand_cmd[4]=0x00;hand_cmd[5]=0x55;
	ser_hand.write(hand_cmd,6);//reset the hand*/

    /*sleep(5);                                            
	joint_angle_data[1]=0x04;
	ser_arm.write(joint_angle_data,18);//reset the arm*/

    /*sleep(2);
    hand_cmd[1]=0x01;hand_cmd[2]=0x06;hand_cmd[3]=0x00;hand_cmd[4]=0x00;hand_cmd[5]=0x55;
	ser_hand.write(hand_cmd,6);//reset the hand*/
}
