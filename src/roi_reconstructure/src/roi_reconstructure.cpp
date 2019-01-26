#include "../include/roi_reconstructure/roi_reconstructure.h"

int main(int argc, char **argv)
{
    grasp_type=atoi(argv[1]);
    
	ros::init(argc, argv, "reconstructure");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub_left=it.subscribe("scene/left/image_raw",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it.subscribe("scene/right/image_raw",1,ImageCallback_right);

	image_transport::Publisher left_pub= it.advertise("scene/left/image", 1);//for rviz UI
	image_transport::Publisher right_pub= it.advertise("scene/right/image", 1);

	image_transport::Publisher disparity_pub= it.advertise("scene/disparity", 1);//disparity image for rviz UI
	image_transport::Publisher tracking_pub= it.advertise("scene/left/trackingandregression", 1);//tracking image for rviz UI

	ros::Publisher pos_pub=nh.advertise<geometry_msgs::PointStamped>("scene/left/point",1000);
	ros::Subscriber sub=nh.subscribe("/scene/left/fit_point",1000,ImagePoint_callback);//fitted point from gaussian regression

	ros::Subscriber yolo_sub=nh.subscribe("/darknet_ros/bounding_boxes",1000,BoundingBox_callback);

	bm->setPreFilterCap(31);//31
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setTextureThreshold(10);//10
	bm->setUniquenessRatio(15);//15
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);

	sgbm->setPreFilterCap(63);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);

	stereo_calibrate_initialize();

	//int count=1;
	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) 
		{
			//cout<<"no image"<<endl;
			continue;
		}
		//去掉非roi区域
		Mat img1_remap,img2_remap,img1_remap_dst,img2_remap_dst;

		remap(img1_raw, img1_remap, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2_remap, map21, map22, INTER_LINEAR);

	    cv:: cvtColor(img1_remap,img1_remap_dst,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_remap,img2_remap_dst,cv::COLOR_BGR2GRAY);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1_remap_dst.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		//copyMakeBorder(img1_remap_dst, img1_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);//left border
		//copyMakeBorder(img2_remap_dst, img2_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE)


		int64 t=getTickCount();

		//imshow("left", img1_remap_dst);
		//imshow("right", img2_remap_dst);

		bm->compute(img1_remap_dst, img2_remap_dst, disp);//comment it for real time
		//sgbm->compute(img1_remap_dst, img2_remap_dst, disp);

		/*char *left,*right;
		sprintf(left,"/home/x-arm/macaca/data/test/left%d.jpg",count);
		sprintf(right,"/home/x-arm/macaca/data/test/right%d.jpg",count);+
		imwrite(left,img1_remap_dst);
		imwrite(right,img2_remap_dst);
		count++;*/

		const int type = disp.type();
		const uchar depth = type & CV_MAT_DEPTH_MASK;

		if(depth == CV_8U) 
		{
			std::cout<<"cv_8u"<<std::endl;
		} 
		if(depth == CV_16S) 
		{
			std::cout<<"CV_16S"<<std::endl;
		}  

		t=getTickCount()-t;
		printf("Time elapsed by bm: %fms\n",t*1000/getTickFrequency());

		//disp = disp.colRange(numberOfDisparities, img1_remap_dst.cols);

		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		reprojectImageTo3D(disp, xyz, Q, true);
		xyz = xyz * 16;

		//imshow("disparity", (Mat_<uchar>)disp);


		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=MatToPoinXYZ(xyz);

		/*if(cloud!=nullptr)
		{
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud);
		} */

		//viewer.showCloud(cloud); 
		//cout<<"point cloud's size: "<<cloud->points.size()<<endl;

		cout<<"tracking_box: "<<tracking_box<<endl;
		static bool tracking_init_flag=false;
		if(tracking_box.height!=0)
		{
			if(tracking_init_flag)
				cv_tracker->clear();
			cv_tracker = TrackerKCF::create();
			//if(tracking_init_flag)
			//{
				cout<<"tracking_box.height: "<<tracking_box.height<<endl;
				cv_tracker->init(img1_remap,tracking_box);
				tracking_init_flag=true;
			//}
		}
		if(tracking_init_flag)
		{
			cv_tracker->update(img1_remap,tracked_box);
			rectangle(img1_remap,tracked_box,Scalar(255,0,0),2,1);
			circle(img1_remap,gaze_point_array[29],5,Scalar(0,0,255),3,8,0);
			//imshow("Tracking",img1_remap);
			sensor_msgs::ImagePtr tracking_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", img1_remap).toImageMsg();
			tracking_pub.publish(tracking_msg);
		}

		geometry_msgs::PointStamped pos;
		long long sumx=0,sumy=0,sumz=0,count=0;
		//if(detected_object_array.size()>0)
		if(tracked_box.tl().x!=0)
		{
			//cout<<"bounding_box: "<<detected_object_array[0].bounding_box<<endl;
			for(int y=0;y<disp8.rows;y++)
				for(int x=0;x<disp8.cols;x++)
				{
					if(tracked_box.contains(Point2i(x,y))&&xyz.at<Vec3f>(y,x)[2]<800)
					{
						sumx+=xyz.at<Vec3f>(y,x)[0];
						sumy+=xyz.at<Vec3f>(y,x)[1];
						sumz+=xyz.at<Vec3f>(y,x)[2];
						count++;
						//cout<<"xyz: "<<xyz.at<Vec3f>(y,x)<<endl;
					}
						//disp8.at<char>(y,x)=0;
				}
cout<<"point count: "<<count<<endl;
		}

		if(count!=0)
		{
			pos.header.frame_id="lscene_link";
			pos.header.stamp=ros::Time();
			pos.point.x=(sumx/(count))/1000.;
			pos.point.y=(sumy/(count))/1000.;
			pos.point.z=(sumz/(count))/1000.+0.02;
			pos_pub.publish(pos);
			cout<<"pos: "<<endl<<pos<<endl;
		}

		if(detected_object_array.size()>0)
		{
			rectangle(disp8,tracked_box,255,3,8,0);
		}

		sensor_msgs::ImagePtr disparity_msg = cv_bridge::CvImage(std_msgs::Header(),"mono8", disp8).toImageMsg();
		disparity_pub.publish(disparity_msg);

		int c = waitKey(1);

		if (27 == char(c)) break;
	}
}
