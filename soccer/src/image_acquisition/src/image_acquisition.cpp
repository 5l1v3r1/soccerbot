#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"
#include <string.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace cv;

sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img){
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;

    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
                                           (0.0) (1.0) (img->height/2.0)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
                                            (0.0) (1.0) (img->height/2.0) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_input");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::CameraPublisher pub = it.advertiseCamera("/camera_input/image_raw", 1);
    ROS_INFO("Image Acquisition");

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
			ROS_INFO("Testing Image %s", image_names[i].c_str());
			string s = image_names[i];

			size_t found = s.find("._");
			if(found != string::npos)
				s.erase(found, 2);

			frame = imread(s);
//	    	ROS_ERROR_STREAM(s << " " << frame.rows << " " << frame.cols);

	    	msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
	    	cam_info = get_default_camera_info_from_image(msg);

			pub.publish(*msg, cam_info, ros::Time::now());
			ros::Duration(0.2).sleep();
		}
		return EXIT_SUCCESS;
    }
    else {
    	// Open the camera
		std::string streamProvider = "0";
		n.getParam("video_stream_provider", streamProvider);
		VideoCapture cap(streamProvider);

		ROS_INFO("Opened Camera, Starting to publish");


		// Camera Information
		Mat frame;
		sensor_msgs::ImagePtr msg;
		std_msgs::Header header;
		header.frame_id = 1;

		// More info
		camera_info_manager::CameraInfoManager cam_info_manager(n, "camera", "");
		sensor_msgs::CameraInfo cam_info_msg = cam_info_manager.getCameraInfo();

		ros::Rate r(30);
		while (n.ok()) {
			cap >> frame;
			if (pub.getNumSubscribers() > 0) {
				if (!frame.empty()) {
					msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
					cam_info_msg = get_default_camera_info_from_image(msg);
					pub.publish(*msg, cam_info_msg, ros::Time::now());
				}
			}
			r.sleep();
		}
    }
}
