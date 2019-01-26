#include "../include/error_analysis.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "error_analysis_node"); 
    ros::NodeHandle nh; 

	image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub_left=it.subscribe("scene/left/image_raw",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it.subscribe("scene/right/image_raw",1,ImageCallback_right);

	ros::Publisher pos_pub=nh.advertise<geometry_msgs::PointStamped>("scene/left/point",1000);

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	stereo_calibrate_initialize();

	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) 
		{
			//cout<<"no image"<<endl;
			continue;
		}
		Mat img1_remap,img2_remap;

		remap(img1_raw, img1_remap, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2_remap, map21, map22, INTER_LINEAR);

	    Ptr<aruco::Dictionary> dictionary =aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0));

	    std::vector<int> l_ids,r_ids; 
	    std::vector<std::vector<cv::Point2f>> l_corners,r_corners; 
	    cv::aruco::detectMarkers(img1_remap, dictionary, l_corners, l_ids);
	    cv::aruco::detectMarkers(img2_remap, dictionary, r_corners, r_ids);

	    cv::Point l_center(0,0),r_center(0,0);
	    if ( l_ids.size() > 0) {
		    cv::aruco::drawDetectedMarkers(img1_remap, l_corners, l_ids);

		    l_center.x=l_corners[0][0].x;
		    l_center.y=l_corners[0][0].y;
	    }
	    if ( r_ids.size() > 0) {
		    cv::aruco::drawDetectedMarkers(img2_remap, r_corners, r_ids);

		    r_center.x=r_corners[0][0].x;
		    r_center.y=r_corners[0][0].y;
	    }
        imshow("l_aruco",img1_remap);
        imshow("r_aruco",img2_remap);
        waitKey(20);
        if (l_center.x != 0 && r_center.x != 0) 
        {
            disp.at<short>(l_center.y,l_center.x)=(short)(16*(l_center.x-r_center.x));//be careful of the order of x,y!!!///!!!
			cout<<"current pos of tag in left image coordinate: "<<l_center<<endl;
			cout<<"current diaparity of tag:"<<l_center.x-r_center.x<<endl;

    		reprojectImageTo3D(disp, xyz, Q, true);
	        xyz = xyz * 16;

			cout<<"The tag is at ["<< xyz.at<Vec3f>(l_center.y,l_center.x)[0]<<","<<xyz.at<Vec3f>(l_center.y,l_center.x)[1]<<","<<xyz.at<Vec3f>(l_center.y,l_center.x)[2]<<"]"<<endl;

    		geometry_msgs::PointStamped pos;

			pos.header.frame_id="lscene_link";
			pos.header.stamp=ros::Time();
			pos.point.x=xyz.at<Vec3f>(l_center.y,l_center.x)[0]/1000.;
			pos.point.y=xyz.at<Vec3f>(l_center.y,l_center.x)[1]/1000.;
			pos.point.z=xyz.at<Vec3f>(l_center.y,l_center.x)[2]/1000.;
			pos_pub.publish(pos);
			cout<<"pos of tag in lscene_link: "<<endl<<pos<<endl;

		    while(ball_robot.point.x==0)
        		ros::spinOnce();//waiting until ball detected

            cout<<"ball pos in robot"<<ball_robot<<endl;
        }
    }
}
