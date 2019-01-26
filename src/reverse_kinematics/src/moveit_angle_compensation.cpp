#include "../include/reverse_kinematics.h"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define pi 3.1415926
#define armLength 66 //cm

int main(int argc,char** argv)
{
    ros::init(argc, argv, "reverse_kinematics_node"); 
    ros::NodeHandle nh; 

    ros::ServiceClient client = nh.serviceClient<reverse_kinematics::querySolutions>("query_solution");
    reverse_kinematics::querySolutions srv;

	Ser_Arm_Initialize();
    //Ser_Hand_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

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

    cv::Point3d bottleMajorAxis(-1,0,0);

    cv::Point3d graspOffsetDirection,calibOffsetDirection;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveItErrorCode calibSuccess,graspSuccess;
	moveit::planning_interface::MoveGroupInterface::Plan calibPlan,graspPlan;
	geometry_msgs::Pose calibPose,graspPose;

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::MoveGroupInterface group("x_arm");
	group.setPlanningTime(0.1);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//grasp--two steps method
    if(argc==2)
	{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected

		ball_robot_sended=ball_robot;
        ball_robot_sended.point.x+=0.01;
        //ball_robot_sended.point.z+=0.015;

		cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;    //unit : m



//get reachable pose from the pose matrix
        //int idx[]={(int)(100*ball_robot_sended.point.x)+poseMatrixCenter, (int)(100*ball_robot_sended.point.y)+poseMatrixCenter, (int)(100*ball_robot_sended.point.z)+poseMatrixCenter,0};

        srv.request.x = (int)(100*ball_robot_sended.point.x);
        srv.request.y = (int)(100*ball_robot_sended.point.y);
        srv.request.z = (int)(100*ball_robot_sended.point.z);
        if (client.call(srv))
        {
            cout<<"solutions.size(): "<<srv.response.temp.solutions.size()<<endl;
        }
         

        double distanceToBase;

        distanceToBase = sqrt(pow(ball_robot_sended.point.x,2) + pow(ball_robot_sended.point.y,2) + pow(ball_robot_sended.point.z,2));
        if(!( distanceToBase < armLength && srv.response.temp.solutions.size()> 0))
        {
			cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
            cout<<"target pose could not be planned!"<<endl;
            return 0;
        }
		cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
        
        tf2::Quaternion oritationBeforeOffset,oritationGrasp;


//chose the best grasp pose for offset direction
        double loss=1000;
        for(int i = 0;i<srv.response.temp.solutions.size();i++)
        {
            geometry_msgs::Pose tempGraspPose;
            moveit::planning_interface::MoveItErrorCode tempGraspSuccess;
            moveit::planning_interface::MoveGroupInterface::Plan tempGraspPlan;
            tempGraspPose.orientation.w = srv.response.temp.solutions[i].w;
            tempGraspPose.orientation.x = srv.response.temp.solutions[i].x;
            tempGraspPose.orientation.y = srv.response.temp.solutions[i].y;
            tempGraspPose.orientation.z = srv.response.temp.solutions[i].z;
            /*tempGraspPose.orientation.w = 0.16194257;
            tempGraspPose.orientation.x = -0.12426291;
            tempGraspPose.orientation.y = 0.97057;
            tempGraspPose.orientation.z = 0.12777796;*/
            tempGraspPose.position.x = ball_robot_sended.point.x;
            tempGraspPose.position.y = ball_robot_sended.point.y;
            tempGraspPose.position.z = ball_robot_sended.point.z;
            /*tempGraspPose.position.x = -0.58;
            tempGraspPose.position.y = -0.33;
            tempGraspPose.position.z = 0.01;*/

            //cout<<"w: "<<srv.response.temp.solutions[i].w<<"x: "<<srv.response.temp.solutions[i].x<<"y: "<<srv.response.temp.solutions[i].y<<"z: "<<srv.response.temp.solutions[i].z<<endl;
            //cout<<"x: "<<tempGraspPose.position.x<<"y "<<tempGraspPose.position.y<<"z: "<<tempGraspPose.position.z<<endl;


            group.setPoseTarget(tempGraspPose);
            tempGraspSuccess = group.plan(tempGraspPlan);

            if( tempGraspSuccess)
            {
                //cout<<"openrave database success"<<endl;
                tf2::Quaternion targetPoseOritation;
                targetPoseOritation.setW(srv.response.temp.solutions[i].w);
                targetPoseOritation.setX(srv.response.temp.solutions[i].x);
                targetPoseOritation.setY(srv.response.temp.solutions[i].y);
                targetPoseOritation.setZ(srv.response.temp.solutions[i].z);
                tf2::Matrix3x3 targetPoseRotationMatrix(targetPoseOritation);
                cv::Point3d handAxisY(targetPoseRotationMatrix.getColumn(1).x(),targetPoseRotationMatrix.getColumn(1).y(),targetPoseRotationMatrix.getColumn(1).z());   //y axis of hand grasp frame
                double tempLoss;
                //cout<<"targetPoseRotationMatrix.getColumn(2).y()"<<targetPoseRotationMatrix.getColumn(2).y()<<endl;

                switch(graspedObjectType)
                {
                case 0 :   //grasp ball
                    tempLoss = allAngleJointLoss(tempGraspPlan,targetPoseRotationMatrix);
                    if(tempLoss<loss)
                    {
                        //cout<<"----targetPoseRotationMatrix.getColumn(2).y()"<<targetPoseRotationMatrix.getColumn(2).y()<<endl;
                        double xOffset = 0;  //unit : m
                        double zOffset = 0.01;  //unit : m
                        graspOffsetDirection.x = targetPoseRotationMatrix.getColumn(0).x()*xOffset + targetPoseRotationMatrix.getColumn(2).x()*zOffset;
                        graspOffsetDirection.y = targetPoseRotationMatrix.getColumn(0).y()*xOffset + targetPoseRotationMatrix.getColumn(2).y()*zOffset;
                        graspOffsetDirection.z = targetPoseRotationMatrix.getColumn(0).z()*xOffset + targetPoseRotationMatrix.getColumn(2).z()*zOffset;
                        oritationBeforeOffset=targetPoseOritation;
                        loss = tempLoss;
                    }
                    break;

                case 1:   //grasp bottle
                    tempLoss = -bottleMajorAxis.ddot(handAxisY);
                    if(tempLoss<loss)
                    {
                        double xOffset = 0;  //unit : m
                        double zOffset = 0.01;  //unit : m
                        graspOffsetDirection.x = targetPoseRotationMatrix.getColumn(0).x()*xOffset + targetPoseRotationMatrix.getColumn(2).x()*zOffset;
                        graspOffsetDirection.y = targetPoseRotationMatrix.getColumn(0).y()*xOffset + targetPoseRotationMatrix.getColumn(2).y()*zOffset;
                        graspOffsetDirection.z = targetPoseRotationMatrix.getColumn(0).z()*xOffset + targetPoseRotationMatrix.getColumn(2).z()*zOffset;
                        oritationBeforeOffset=targetPoseOritation;
                        loss = tempLoss;
                    }
                    break;
                }
            }
        }

//chose the best grasping pose with grasp offset
        srv.request.x = (int)(100*(ball_robot_sended.point.x+graspOffsetDirection.x));
        srv.request.y = (int)(100*(ball_robot_sended.point.y+graspOffsetDirection.y));
        srv.request.z = (int)(100*(ball_robot_sended.point.z+graspOffsetDirection.z));

        if (client.call(srv))
        {
            cout<<"solutions.size(): "<<srv.response.temp.solutions.size()<<endl;
        }

        double graspDistanceToBase;

        graspDistanceToBase =sqrt(pow(ball_robot_sended.point.x+graspOffsetDirection.x,2)+pow(ball_robot_sended.point.y+graspOffsetDirection.y,2)+pow(ball_robot_sended.point.z+graspOffsetDirection.z,2));
        if(!( graspDistanceToBase < armLength && srv.response.temp.solutions.size() > 0))
        {
            cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
            cout<<"target pose could not be planned!"<<endl;
            return 0;
        }
        cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
        double graspLoss=1000;
        for(int i = 0;i<srv.response.temp.solutions.size();i++)
        {
            geometry_msgs::Pose tempGraspPose;
            moveit::planning_interface::MoveItErrorCode tempGraspSuccess;
            moveit::planning_interface::MoveGroupInterface::Plan tempGraspPlan;
            tempGraspPose.orientation.w = srv.response.temp.solutions[i].w;
            tempGraspPose.orientation.x = srv.response.temp.solutions[i].x;
            tempGraspPose.orientation.y = srv.response.temp.solutions[i].y;
            tempGraspPose.orientation.z = srv.response.temp.solutions[i].z;
            tempGraspPose.position.x = ball_robot_sended.point.x+graspOffsetDirection.x;
            tempGraspPose.position.y = ball_robot_sended.point.y+graspOffsetDirection.y;
            tempGraspPose.position.z = ball_robot_sended.point.z+graspOffsetDirection.z;

            group.setPoseTarget(tempGraspPose);
            tempGraspSuccess = group.plan(tempGraspPlan);

            if( tempGraspSuccess)
            {
                tf2::Quaternion targetPoseOritation;
                targetPoseOritation.setW(srv.response.temp.solutions[i].w);
                targetPoseOritation.setX(srv.response.temp.solutions[i].x);
                targetPoseOritation.setY(srv.response.temp.solutions[i].y);
                targetPoseOritation.setZ(srv.response.temp.solutions[i].z);
                tf2::Matrix3x3 targetPoseRotationMatrix(targetPoseOritation);
                cv::Point3d handAxisY(targetPoseRotationMatrix.getColumn(1).x(),targetPoseRotationMatrix.getColumn(1).y(),targetPoseRotationMatrix.getColumn(1).z());   //y axis of hand grasp frame
                double tempLoss;

                switch(graspedObjectType)
                {
                case 0 :   //grasp ball
                    tempLoss = distanceBetweenQuaterions(targetPoseOritation,oritationBeforeOffset);
                    if(tempLoss<graspLoss)
                    {
                        double xOffset = 0;  //unit : m
                        double zOffset = 0.08;  //unit : m
                        graspPose.orientation.w = srv.response.temp.solutions[i].w;
                        graspPose.orientation.x = srv.response.temp.solutions[i].x;
                        graspPose.orientation.y = srv.response.temp.solutions[i].y;
                        graspPose.orientation.z = srv.response.temp.solutions[i].z;
                        graspPose.position.x = ball_robot_sended.point.x+graspOffsetDirection.x;
                        graspPose.position.y = ball_robot_sended.point.y+graspOffsetDirection.y;
                        graspPose.position.z = ball_robot_sended.point.z+graspOffsetDirection.z;
                        graspLoss = tempLoss;
                        calibOffsetDirection.x = targetPoseRotationMatrix.getColumn(0).x()*xOffset + targetPoseRotationMatrix.getColumn(2).x()*zOffset;
                        calibOffsetDirection.y = targetPoseRotationMatrix.getColumn(0).y()*xOffset + targetPoseRotationMatrix.getColumn(2).y()*zOffset;
                        calibOffsetDirection.z = targetPoseRotationMatrix.getColumn(0).z()*xOffset + targetPoseRotationMatrix.getColumn(2).z()*zOffset;
                        oritationGrasp = targetPoseOritation;

                    }
                    break;

                case 1:   //grasp bottle
                    tempLoss = -bottleMajorAxis.ddot(handAxisY);
                    if(tempLoss<graspLoss)
                    {
                        double xOffset = -0.06;  //unit : m
                        double zOffset = 0.08;  //unit : m
                        graspPose.orientation.w = srv.response.temp.solutions[i].w;
                        graspPose.orientation.x = srv.response.temp.solutions[i].x;
                        graspPose.orientation.y = srv.response.temp.solutions[i].y;
                        graspPose.orientation.z = srv.response.temp.solutions[i].z;
                        graspPose.position.x = ball_robot_sended.point.x+graspOffsetDirection.x;
                        graspPose.position.y = ball_robot_sended.point.y+graspOffsetDirection.y;
                        graspPose.position.z = ball_robot_sended.point.z+graspOffsetDirection.z;
                        graspLoss = tempLoss;
                        calibOffsetDirection.x = targetPoseRotationMatrix.getColumn(0).x()*xOffset + targetPoseRotationMatrix.getColumn(2).x()*zOffset;
                        calibOffsetDirection.y = targetPoseRotationMatrix.getColumn(0).y()*xOffset + targetPoseRotationMatrix.getColumn(2).y()*zOffset;
                        calibOffsetDirection.z = targetPoseRotationMatrix.getColumn(0).z()*xOffset + targetPoseRotationMatrix.getColumn(2).z()*zOffset;
                        oritationGrasp = targetPoseOritation;
                    }
                    break;
                }
            }
        }

        cout<<"graspOffsetDirection: "<<graspOffsetDirection.x<<" "<<graspOffsetDirection.y<<" "<<graspOffsetDirection.z<<endl;
        cout<<"graspPose: "<<graspPose.orientation.w<<" "<<graspPose.orientation.x<<" "<<graspPose.orientation.y<<" "<<graspPose.orientation.z<<" "<<graspPose.position.x<<" "<<graspPose.position.y<<" "<<graspPose.position.z<<endl;

//chose the best calib pose
        srv.request.x = (int)(100*(graspPose.position.x+calibOffsetDirection.x));
        srv.request.y = (int)(100*(graspPose.position.y+calibOffsetDirection.y));
        srv.request.z = (int)(100*(graspPose.position.z+calibOffsetDirection.z));

        if (client.call(srv))
        {
            cout<<"solutions.size(): "<<srv.response.temp.solutions.size()<<endl;
        }

        double calibDistanceToBase;

        calibDistanceToBase =sqrt(pow(graspPose.position.x+calibOffsetDirection.x,2)+pow(graspPose.position.y+calibOffsetDirection.y,2)+pow(graspPose.position.z+calibOffsetDirection.z,2));
        if(!( calibDistanceToBase < armLength && srv.response.temp.solutions.size() > 0))
        {
            cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
            cout<<"target pose could not be planned!"<<endl;
            return 0;
        }
        cout<<"matrix index: "<<srv.request.x<<" "<<srv.request.y<<" "<<srv.request.z<<", number of solutions: "<<srv.response.temp.solutions.size()<<endl;
        double calibLoss=1000;
        for(int i = 0;i<srv.response.temp.solutions.size();i++)
        {
            geometry_msgs::Pose tempGraspPose;
            moveit::planning_interface::MoveItErrorCode tempGraspSuccess;
            moveit::planning_interface::MoveGroupInterface::Plan tempGraspPlan;
            tempGraspPose.orientation.w = srv.response.temp.solutions[i].w;
            tempGraspPose.orientation.x = srv.response.temp.solutions[i].x;
            tempGraspPose.orientation.y = srv.response.temp.solutions[i].y;
            tempGraspPose.orientation.z = srv.response.temp.solutions[i].z;
            tempGraspPose.position.x = graspPose.position.x+calibOffsetDirection.x;
            tempGraspPose.position.y = graspPose.position.y+calibOffsetDirection.y;
            tempGraspPose.position.z = graspPose.position.z+calibOffsetDirection.z;

            group.setPoseTarget(tempGraspPose);
            tempGraspSuccess = group.plan(tempGraspPlan);

            if( tempGraspSuccess)
            {
                tf2::Quaternion targetPoseOritation;
                targetPoseOritation.setW(srv.response.temp.solutions[i].w);
                targetPoseOritation.setX(srv.response.temp.solutions[i].x);
                targetPoseOritation.setY(srv.response.temp.solutions[i].y);
                targetPoseOritation.setZ(srv.response.temp.solutions[i].z);
                tf2::Matrix3x3 targetPoseRotationMatrix(targetPoseOritation);
                cv::Point3d handAxisY(targetPoseRotationMatrix.getColumn(1).x(),targetPoseRotationMatrix.getColumn(1).y(),targetPoseRotationMatrix.getColumn(1).z());   //y axis of hand grasp frame
                double tempLoss;

                switch(graspedObjectType)
                {
                case 0 :   //grasp ball
                    tempLoss = distanceBetweenQuaterions(targetPoseOritation,oritationGrasp);
                    if(tempLoss<calibLoss)
                    {
                        calibPose.orientation.w = srv.response.temp.solutions[i].w;
                        calibPose.orientation.x = srv.response.temp.solutions[i].x;
                        calibPose.orientation.y = srv.response.temp.solutions[i].y;
                        calibPose.orientation.z = srv.response.temp.solutions[i].z;
                        calibPose.position.x = graspPose.position.x+calibOffsetDirection.x;
                        calibPose.position.y = graspPose.position.y+calibOffsetDirection.y;
                        calibPose.position.z = graspPose.position.z+calibOffsetDirection.z;
                        calibLoss = tempLoss;

                    }
                    break;

                case 1:   //grasp bottle
                    tempLoss = -bottleMajorAxis.ddot(handAxisY);
                    if(tempLoss<calibLoss)
                    {
                        calibPose.orientation.w = srv.response.temp.solutions[i].w;
                        calibPose.orientation.x = srv.response.temp.solutions[i].x;
                        calibPose.orientation.y = srv.response.temp.solutions[i].y;
                        calibPose.orientation.z = srv.response.temp.solutions[i].z;
                        calibPose.position.x = graspPose.position.x+calibOffsetDirection.x;
                        calibPose.position.y = graspPose.position.y+calibOffsetDirection.y;
                        calibPose.position.z = graspPose.position.z+calibOffsetDirection.z;
                        calibLoss = tempLoss;
                    }
                    break;
                }
            }
        }
        cout<<"calibOffsetDirection: "<<calibOffsetDirection.x<<" "<<calibOffsetDirection.y<<" "<<calibOffsetDirection.z<<endl;
        cout<<"calibPose: "<<calibPose.orientation.w<<" "<<calibPose.orientation.x<<" "<<calibPose.orientation.y<<" "<<calibPose.orientation.z<<" "<<calibPose.position.x<<" "<<calibPose.position.y<<" "<<calibPose.position.z<<endl;


// step1 calib
        group.setPoseTarget(calibPose);
        calibSuccess = group.plan(calibPlan);

		cout << "angle step 1: "\
			<< calibPlan.trajectory_.joint_trajectory.points.back().positions.at(0) << " " << calibPlan.trajectory_.joint_trajectory.points.back().positions.at(1) << " "\
			<< calibPlan.trajectory_.joint_trajectory.points.back().positions.at(2) << " " << calibPlan.trajectory_.joint_trajectory.points.back().positions.at(3) << " "\
			<< calibPlan.trajectory_.joint_trajectory.points.back().positions.at(4) << " " << calibPlan.trajectory_.joint_trajectory.points.back().positions.at(5) << " "\
			<< calibPlan.trajectory_.joint_trajectory.points.back().positions.at(6) << endl;

		short int theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(0)*1000);//plus 1000 to reserve three digit after the dot.
		unsigned char* temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
		joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(1)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(2)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(3)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(4)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(5)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

		theta=(short int)(calibPlan.trajectory_.joint_trajectory.points.back().positions.at(6)*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

        for(int i=0;i<18;i++)
            calib_angle_data[i]=joint_angle_data[i];

		//cout<<"input something to confirm ball's pos"<<endl;
		char c;
		//cin>>c;

		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"data sended to serial: "<<i<<endl;
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
    moveit::planning_interface::MoveGroupInterface groupCalib("x_arm_calib");
	group.setPlanningTime(0.1);

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

		/*for(int i=0;i<3;i++)
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
                    groupCalib.setPoseTarget(target_pose);
                    success = groupCalib.plan(my_plan);
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
        groupCalib.setPoseTarget(opti_target_pose);*/

		target_pose.orientation.w = joint6_robot.getRotation().getW();
		target_pose.orientation.x = joint6_robot.getRotation().getX();
		target_pose.orientation.y = joint6_robot.getRotation().getY();
		target_pose.orientation.z = joint6_robot.getRotation().getZ();
		target_pose.position.x = joint6_robot.getOrigin().x();
		target_pose.position.y = joint6_robot.getOrigin().y();
		target_pose.position.z = joint6_robot.getOrigin().z();        
        groupCalib.setPoseTarget(target_pose);

        success = groupCalib.plan(my_plan);
		cout<<"optimal ijk: "<<opti_i<<" "<<opti_j<<" "<<opti_k<<endl;
		cout<<"angle: "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(5)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(6)<<" "<<endl;
		cout<<"angle error: "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(0)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(1)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(2)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(3)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(4)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(5)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(5)<<" "<<my_plan.trajectory_.joint_trajectory.points.back().positions.at(6)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(6)<<" "<<endl;
	}
	catch (tf::TransformException &ex)
	{
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
	}

	//cout<<"input something to confirm the error"<<endl;
	char c;
	//cin>>c;

    group.setPoseTarget(graspPose);
    graspSuccess = group.plan(graspPlan);

	cout << "angle step 2: "\
		<< graspPlan.trajectory_.joint_trajectory.points.back().positions.at(0) << " " << graspPlan.trajectory_.joint_trajectory.points.back().positions.at(1) << " "\
		<< graspPlan.trajectory_.joint_trajectory.points.back().positions.at(2) << " " << graspPlan.trajectory_.joint_trajectory.points.back().positions.at(3) << " "\
		<< graspPlan.trajectory_.joint_trajectory.points.back().positions.at(4) << " " << graspPlan.trajectory_.joint_trajectory.points.back().positions.at(5) << " "\
		<< graspPlan.trajectory_.joint_trajectory.points.back().positions.at(6) << endl;

	cout<<"step 3: make up for the error!!!"<<endl;

	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(0)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(0)-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(0)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(0));
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(1)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(1)-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(1)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(1));
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(2)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(2)-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(2)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(2));
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(3)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(3)-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(3)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(3));
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(4)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(4)-(my_plan.trajectory_.joint_trajectory.points.back().positions.at(4)-calibPlan.trajectory_.joint_trajectory.points.back().positions.at(4));
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(5)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(5);
	graspPlan.trajectory_.joint_trajectory.points.back().positions.at(6)=graspPlan.trajectory_.joint_trajectory.points.back().positions.at(6);

	short int theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(0)*1000);//plus 1000 to reserve three digit after the dot.
	unsigned char* temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
	joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(1)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(2)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(3)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(4)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(5)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

	theta=(short int)(graspPlan.trajectory_.joint_trajectory.points.back().positions.at(6)*1000);
	temp=(unsigned char *)&theta;
	joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];

	size_t i=ser_arm.write(joint_angle_data,18);
	cout<<"data sended to serial: "<<i<<endl;

    
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
