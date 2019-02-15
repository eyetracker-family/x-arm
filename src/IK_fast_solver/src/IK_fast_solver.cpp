#include "../include/IK_fast_solver.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "IK_fast_solver_node"); 
    ros::NodeHandle nh; 

    ros::ServiceClient grasp_client = nh.serviceClient<IK_fast_solver::grasp>("grasp");
    IK_fast_solver::grasp grasp_srv;

	Ser_Arm_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_listener)));

	joint_angle_data[0]=0x55;joint_angle_data[1]=0x0b;joint_angle_data[2]=0x05;joint_angle_data[3]=0x0e;//ball pos data header

	if(argc==1)//reset
	{
		joint_angle_data[1]=0x04;
		ser_arm.write(joint_angle_data,18);//reset the arm
		cout<<"arm and hand reseted!!!"<<endl;
		return 0;
	}

    int graspedObjectType=0; // 0:ball, 1:bottle
    graspedObjectType=atoi(argv[1]);

    //if(argc==2)//grasp--two steps method
	//{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected

        if(atoi(argv[1])==1)
        {
            //trajectory planning
            set_joint_angle_const(joint_angle_data,0.8,0.46,-0.16,1.1,-0.35,0,0);
	        ser_arm.write(joint_angle_data,18);
	        cout<<"trajectory planning data sended to serial: "<<endl;
        }

		ball_robot_sended=ball_robot;
        ball_robot_sended.point.x+=0.015;
        //ball_robot_sended.point.z+=0.015;
		cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;//unit : m



        // step1 pre
        grasp_srv.request.flag=0;
        grasp_srv.request.x=ball_robot_sended.point.x;
        grasp_srv.request.y=ball_robot_sended.point.y;
        grasp_srv.request.z=ball_robot_sended.point.z;
        if(atoi(argv[1])==1)
        {
            grasp_srv.request.q_w=0.5;
            grasp_srv.request.q_x=0.5;
            grasp_srv.request.q_y=-0.5;
            grasp_srv.request.q_z=0.5;
        }
        if(atoi(argv[1])==0)
        {
            grasp_srv.request.q_w=0;
            grasp_srv.request.q_x=0;
            grasp_srv.request.q_y=0;
            grasp_srv.request.q_z=0;
        }

        grasp_client.call(grasp_srv);
		cout << "step1 target angle calib: "<< grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "<< grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "<< grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "<< grasp_srv.response.angle6 << endl;
        set_joint_angle(joint_angle_data,grasp_srv);
        for(int i=0;i<18;i++)
            calib_angle_data[i]=joint_angle_data[i];//for pulling out the ball
		cout<<"input something to confirm ball's pos and step1 angle"<<endl;
		cin>>c;
		ser_arm.write(joint_angle_data,18);

        // step2 calib the error
	    sleep(6);//sleep 6 seconds to ensure step1 executed(the prerequest of caliberation).
	    cout<<"step 2: calib the error!!!"<<endl;
	    try
	    {
		    tf_listener.lookupTransform("robot_link", "joint6_link",ros::Time(0), joint6_robot);//joint6_robot is the transform from frame /joint6_link to frame /robot_link.
            grasp_srv.request.flag=1;
            grasp_srv.request.x=joint6_robot.getOrigin().x();
            grasp_srv.request.y=joint6_robot.getOrigin().y();
            grasp_srv.request.z=joint6_robot.getOrigin().z();
            grasp_srv.request.q_w=joint6_robot.getRotation().getW();
            grasp_srv.request.q_x=joint6_robot.getRotation().getX();
            grasp_srv.request.q_y=joint6_robot.getRotation().getY();
            grasp_srv.request.q_z=joint6_robot.getRotation().getZ();
            grasp_client.call(grasp_srv);

	        /*cout << "angle step 2: "<< grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "<< grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "<< grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "<< grasp_srv.response.angle6 << endl;

	        cout<<"input something to confirm the error"<<endl;
	        cin>>c;

            set_joint_angle(joint_angle_data,grasp_srv);
		    ser_arm.write(joint_angle_data,18);
            sleep(5);*/
	    }
	    catch (tf::TransformException &ex)
	    {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
	    }


        // step3 grasp
        grasp_srv.request.flag=2;
        grasp_srv.request.x=ball_robot_sended.point.x;
        grasp_srv.request.y=ball_robot_sended.point.y;
        grasp_srv.request.z=ball_robot_sended.point.z;
        grasp_srv.request.q_w=0;
        grasp_srv.request.q_x=0;
        grasp_srv.request.q_y=0;
        grasp_srv.request.q_z=0;
        grasp_client.call(grasp_srv);
	    cout << "angle step 3: "<< grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "<< grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "<< grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "<< grasp_srv.response.angle6 << endl;
        set_joint_angle(joint_angle_data,grasp_srv);
	    ser_arm.write(joint_angle_data,18);
	    cout<<"ready for grasp!!!"<<endl;

        sleep(8);
        if(atoi(argv[1])==1)
        {
            grasp_srv.request.flag=3;
            grasp_srv.request.x=ball_robot_sended.point.x-0.1;
            grasp_srv.request.y=ball_robot_sended.point.y;
            grasp_srv.request.z=ball_robot_sended.point.z;
            grasp_srv.request.q_w=0.5;
            grasp_srv.request.q_x=0.5;
            grasp_srv.request.q_y=-0.5;
            grasp_srv.request.q_z=0.5;
            grasp_client.call(grasp_srv);
            set_joint_angle(joint_angle_data,grasp_srv);
	        ser_arm.write(joint_angle_data,18);

            //drink water
            sleep(5);
            set_joint_angle_const(joint_angle_data,-0.04,-0.35,1.41,2,0.51,0.44,-0.35);
            ser_arm.write(joint_angle_data,18);
            cout<<"drink water "<<endl;
        }
        if(atoi(argv[1])==0&&argc==3)
        {
	        ser_arm.write(calib_angle_data,18);
        }
	//}


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
