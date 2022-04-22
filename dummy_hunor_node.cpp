#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "can_msgs/Frame.h"
#include "std_msgs/String.h"


class dummy
{
private:
	image_transport::Subscriber left_sub; //left and right image subscribers
	image_transport::Subscriber right_sub;

	image_transport::Publisher imgPub;
	ros::Publisher topicPub;

	cv::Mat cvImgL; //left and right images
	cv::Mat cvImgR;

	double camera_fps;
	double camera_time = 0; //current time on each iteration

public:
	dummy(int argc, char** argv)
	{
		ros::init(argc, argv, "dummy_hunor_node");

		//Sub

		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);

		left_sub = it.subscribe("/left/image_rect_color/raw", 1, callback); 
		right_sub = it.subscribe("/right/image_rect_color/raw", 1, callback);

		message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_sub, right_sub, 10);
		sync.registerCallback(boost::bind(&callback, _1, _2);

		
		//Pub
		ros::NodeHandle n1;
		topicPub = n1.advertise<can_msgs::Frame>("/sent_messages", 1000);
		ros::Timer timerSent = n1.createTimer(ros::Duration(1 / 50), sentPub);

		ros::spin();
	}
	~dummy();

	void callback(const sensor_msgs::Image::ConstPtr& img_left, const sensor_msgs::Image::ConstPtr& img_right)
	{
		//concatenation

		//conversion
		try
		{
			cvImgL = cv_bridge::toCvCopy(img_left, "bgr8")->image;
			cvImgR = cv_bridge::toCvCopy(img_right, "bgr8")->image;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		}

		//dimensions
		int rows = img_left->height;
		int cols = img_left->width + img_right->width;

		//black image in those dimensions
		cv::Mat3b result(rows, cols, cv::Vec3b(0, 0, 0)); 

		//copying to black image side by side
		cvImgL.copyTo(result(cv::Rect(0, 0, img_left->width, img_left->height))); 
		cvImgR.copyTo(result(cv::Rect(img_left->width, 0, img_right->width, img_right->height)));

		//image Pub
		ros::NodeHandle n0;
		image_transport::ImageTransport it0(n0);

		imgPub = it0.advertise("/dummy_img_hunor", 1);
		
		sensor_msgs::ImagePtr out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg(); 

		while (n0.ok())
		{
			imgPub.publish(out);
		}
		
		if (camera_time == 0) camera_fps = 0; //1. callback
		
		int tmp = camera_time;
		camera_time = ros::Time::now();
		if (camera_time != 0) camera_fps = 1.0 / (camera_time - tmp);

	}

	void sentPub(const ros::TimerEvent&) 
	{
		while (ros::ok())
		{
			can_msgs::Frame msg;

			msg.id = 0x414;
			msg.is_rtr = 0;
			msg.is_extended = 0;
			msg.is_error = 0;
			msg.dlc = 8;
			msg.data[0] = camera_fps;
			for (int i = 1; i < 8; i++)
			{
				msg.data[i] = 0;
			}

			topicPub.publish(msg);
		}
	}
};





int main(int argc, char **argv)
{
	dummy(argc, argv);
	
	return 0;
}