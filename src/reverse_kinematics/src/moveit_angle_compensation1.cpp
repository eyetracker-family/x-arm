#include "../include/reverse_kinematics.h"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>

/*#define X_center 70
#define Y_center 70
#define Z_center 0
#define X_range 140
#define Y_range 140
#define Z_range 70*/
#define pi 3.1415926
#define armLength 66 //cm



int main(int argc,char** argv)
{
    ros::init(argc, argv, "reverse_kinematics_node"); 
    ros::NodeHandle nh; 

	Ser_Arm_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	joint_angle_data[0]=0x55;joint_angle_data[1]=0x0b;joint_angle_data[2]=0x05;joint_angle_data[3]=0x0e;//ball pos data header

	if(argc==2)//reset
	{
		joint_angle_data[1]=0x04;
		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"arm reseted!!!"<<endl;
		return 0;
	}

	std::string fileName="/home/x-arm/macaca/data/reachability_1364698_576.YAML";
	int sz[]={X_range,Y_range,Z_range,4}; //4: yaw pitch roll reachable[0,1]
//	cv::Mat poseMatrix(4,sz,CV_64F,cv::Scalar(0));
    cv::Mat poseMatrix,reachabilitySet;
	cv::FileStorage fs;
    int poseMatrixCenter;
	fs.open(fileName,cv::FileStorage::READ);
	fs["poseSolutionsIndexInSerialization"]>>poseMatrix;
	fs["reachabilitySet"]>>reachabilitySet;
	fs["center"]>>poseMatrixCenter;

	double graspOffset=2.0;   //unit: cm
	double calibOffset=10.0;   //unit: cm

	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveItErrorCode calibSuccess,graspSuccess;
	moveit::planning_interface::MoveGroupInterface::Plan calibPlan,graspPlan;
	geometry_msgs::Pose calibPose,graspPose;

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::MoveGroupInterface group("x_arm");
	group.setPlanningTime(0.1);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


	if(argc==1)//grasp two steps method
	{
		while(ball_robot.point.x==0)
    		ros::spinOnce();//waiting until ball detected

		ball_robot_sended=ball_robot;

		cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;    //unit : m
		//cout<<"step1 pos in robot: "<<ball_robot_sended.point.x+0.1<<" "<<ball_robot_sended.point.y-0.1<<" "<<ball_robot_sended.point.z+0.1<<endl;


        //get planned pose from the pose matrix
        int idx[]={(int)(100*ball_robot_sended.point.x)+poseMatrixCenter, (int)(100*ball_robot_sended.point.y)+poseMatrixCenter, (int)(100*ball_robot_sended.point.z+poseMatrixCenter),0};
        double distanceToBase;
        int numPosesOnePoint, poseSolutionStartIndex;
        numPosesOnePoint = poseMatrix.at<double>(idx);
        idx[3] = 1;
        poseSolutionStartIndex = poseMatrix.at<double>(idx);

        distanceToBase = ball_robot_sended.point.x*ball_robot_sended.point.x + ball_robot_sended.point.y*ball_robot_sended.point.y + ball_robot_sended.point.z*ball_robot_sended.point.z;
        if(!( distanceToBase < armLength && numPosesOnePoint > 0))
        {
			cout<<"matrix index: "<<idx[0]<<" "<<idx[1]<<" "<<idx[2]<<", number of solutions: "<<numPosesOnePoint<<endl;
            cout<<"target pose could not be planned!"<<endl;
            return 0;
        }
		cout<<"matrix index: "<<idx[0]<<" "<<idx[1]<<" "<<idx[2]<<", number of solutions: "<<numPosesOnePoint<<endl;
        

        cv::Point3d targetPoseEulerAngle;
        tf2::Quaternion targetPoseOritation;
        idx[3]=0;
        targetPoseEulerAngle.x = poseMatrix.at<double>(idx);
        idx[3]=1;
        targetPoseEulerAngle.y = poseMatrix.at<double>(idx);
        idx[3]=2;
        targetPoseEulerAngle.z = poseMatrix.at<double>(idx);
        targetPoseOritation.setEuler(targetPoseEulerAngle.x,targetPoseEulerAngle.y,targetPoseEulerAngle.z);

        tf2::Matrix3x3 targetPoseRotationMatrix(targetPoseOritation);
        cv::Point3d offsetDirection(targetPoseRotationMatrix.getColumn(2).x(),targetPoseRotationMatrix.getColumn(2).y(),targetPoseRotationMatrix.getColumn(2).z());
		cout<<"offsetDirection x,y,z: "<<offsetDirection.x<<" "<<offsetDirection.y<<" "<<offsetDirection.z<<endl;



        int idxCalib[]={(int)(100*ball_robot_sended.point.x+(calibOffset+graspOffset)*offsetDirection.x)+X_center,(int)(100*ball_robot_sended.point.y+(calibOffset+graspOffset)*offsetDirection.y)+Y_center,(int)(100*ball_robot_sended.point.z+(calibOffset+graspOffset)*offsetDirection.z)+Z_center,3};
		//int idxCalib[]={(int)(100*ball_robot_sended.point.x)+X_center,(int)(100*ball_robot_sended.point.y-15)+Y_center,(int)(100*ball_robot_sended.point.z)+Z_center,3};
        int idxGrasp[]={(int)(100*ball_robot_sended.point.x + graspOffset*offsetDirection.x)+X_center,(int)(100*ball_robot_sended.point.y+graspOffset*offsetDirection.y)+Y_center,(int)(100*ball_robot_sended.point.z+graspOffset*offsetDirection.z)+Z_center,3};

        cv::Point3d graspPoseEulerAngle,calibPoseEulerAngle;
        tf2::Quaternion graspPoseOritation,calibPoseOritation;


        //get the calib pose
        if(!(idxCalib[0]>=0 && idxCalib[0]<X_range && idxCalib[1]>=0 && idxCalib[1]<Y_range && idxCalib[2]>=0 && idxCalib[2]<Z_range && fabs(poseMatrix.at<double>(idxCalib)-1)<0.0001))
        {
            cout<<"calib pose could not be planned!"<<endl;
            return 0;
        }
        idxCalib[3]=0;
        calibPoseEulerAngle.x = poseMatrix.at<double>(idxCalib);
        idxCalib[3]=1;
        calibPoseEulerAngle.y = poseMatrix.at<double>(idxCalib);
        idxCalib[3]=2;
        calibPoseEulerAngle.z = poseMatrix.at<double>(idxCalib);
        calibPoseOritation.setEuler(calibPoseEulerAngle.x,calibPoseEulerAngle.y,calibPoseEulerAngle.z);
        calibPose.orientation.w = calibPoseOritation.w();
        calibPose.orientation.x = calibPoseOritation.x();
        calibPose.orientation.y = calibPoseOritation.y();
        calibPose.orientation.z = calibPoseOritation.z();
        calibPose.position.x = ball_robot_sended.point.x + (calibOffset+graspOffset)*offsetDirection.x/100.0;
        calibPose.position.y = ball_robot_sended.point.y + (calibOffset+graspOffset)*offsetDirection.y/100.0;
        calibPose.position.z = ball_robot_sended.point.z + (calibOffset+graspOffset)*offsetDirection.z/100.0;


        //get the grasp pose
        if(!(idxGrasp[0]>=0 && idxGrasp[0]<X_range && idxGrasp[1]>=0 && idxGrasp[1]<Y_range && idxGrasp[2]>=0 && idxGrasp[2]<Z_range && fabs(poseMatrix.at<double>(idxGrasp)-1)<0.0001))
        {
            cout<<"grasp pose could not be planned!"<<endl;
            return 0;
        }
        idxGrasp[3]=0;
        graspPoseEulerAngle.x = poseMatrix.at<double>(idxGrasp);
        idxGrasp[3]=1;
        graspPoseEulerAngle.y = poseMatrix.at<double>(idxGrasp);
        idxGrasp[3]=2;
        graspPoseEulerAngle.z = poseMatrix.at<double>(idxGrasp);
        graspPoseOritation.setEuler(graspPoseEulerAngle.x,graspPoseEulerAngle.y,graspPoseEulerAngle.z);
        graspPose.orientation.w = graspPoseOritation.w();
        graspPose.orientation.x = graspPoseOritation.x();
        graspPose.orientation.y = graspPoseOritation.y();
        graspPose.orientation.z = graspPoseOritation.z();
        graspPose.position.x = ball_robot_sended.point.x + (graspOffset)*offsetDirection.x/100.0+0.01;
        graspPose.position.y = ball_robot_sended.point.y + (graspOffset)*offsetDirection.y/100.0;
        graspPose.position.z = ball_robot_sended.point.z + (graspOffset)*offsetDirection.z/100.0;

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

		cout<<"input something to confirm ball's pos"<<endl;
		char c;
		cin>>c;

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
        groupCalib.setPoseTarget(opti_target_pose);
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

	cout<<"input something to confirm the error"<<endl;
	char c;
	cin>>c;

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
}
