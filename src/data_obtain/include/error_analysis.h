#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

Mat xyz;
Mat map11, map12, map21, map22;
Size img_size(1280,720);

float scale=1;
Rect roi1, roi2;
Mat Q;
Mat img1_raw,img2_raw;

Mat disp(720,1280,CV_16SC1), disp8;

std::string intrinsic_filename = "/home/x-arm/macaca/data/reconstruction/intrinsics.yml";
std::string extrinsic_filename = "/home/x-arm/macaca/data/reconstruction/extrinsics.yml";

geometry_msgs::PointStamped ball_lscene,ball_robot;

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

void ImageCallback_left(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img1_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        //cv::imshow("left_scene",img1_raw);
        //cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}
void ImageCallback_right(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img2_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        //cv::imshow("right_scene",img2_raw);
        //cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}

void stereo_calibrate_initialize()
{
	FileStorage fs(intrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
		return;
	}

	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= scale;
	M2 *= scale;

	fs.open(extrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
		return;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	cout<<"roi1.size: "<<roi1.size()<<endl;
	cout<<"roi2.size: "<<roi2.size()<<endl;
	cout<<"matrix M1: "<<M1<<endl;
	cout<<"matrix D1: "<<D1<<endl;
	cout<<"matrix R1: "<<R1<<endl;
	cout<<"matrix R2: "<<R2<<endl;
	cout<<"matrix Q: "<<Q<<endl;

	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	//Point2d dst(map11.at<short>(400,400),map12.at<short>(400,400));
}
