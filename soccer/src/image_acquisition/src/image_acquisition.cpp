#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_input");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::CameraPublisher pub = it.advertiseCamera("/camera_input/image_raw", 1);

    bool image_test;
    n.getParam("image_test", image_test);

    if (image_test) {
    	// Folder information
    	string path;
    	n.getParam("image_test_folder", path);
    	String image_path = path;
    	vector<String> image_names;
    	glob(image_path, image_names, false);
    	// Image information
    	Mat frame;
		sensor_msgs::ImagePtr msg;
		sensor_msgs::CameraInfo cam_info;
		std_msgs::Header header;
		header.frame_id = 1;
		for (size_t i = 0; i < image_names.size(); ++i) {
			ROS_INFO("Testing Image %s", image_names[i]);
			frame = imread(image_names[i]);
			msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
			pub.publish(*msg, cam_info, ros::Time::now());
			ros::Duration(0.1).sleep();
		}
    }
    else {
    	// Open the camera
		std::string streamProvider = "0";
		n.getParam("video_stream_provider", streamProvider);
		VideoCapture cap;
		cap.open(-1);
		if (!cap.isOpened()) {
			ROS_ERROR("Could not open the stream.");
			return -1;
		}
		ROS_INFO("Opened Camera, Starting to publish");

		// Camera Information
		Mat frame;
		sensor_msgs::ImagePtr msg;
		sensor_msgs::CameraInfo cam_info;
		std_msgs::Header header;
		header.frame_id = 1;

		ros::Rate r(30);
		while (n.ok()) {
			cap >> frame;
			if (pub.getNumSubscribers() > 0) {
				if (!frame.empty()) {
					msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
					pub.publish(*msg, cam_info, ros::Time::now());
				}
			}
			r.sleep();
		}
    }
}
