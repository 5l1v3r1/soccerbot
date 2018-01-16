#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "../include/point_transform.hpp"
#include <std_msgs/Float32.h>

using namespace std;
using namespace ros;

image_transport::Subscriber raw_image_sub;
Publisher D_pub;

ros::Publisher center_point;
void get_image_size(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	camera_size = img->image.size();
//	ROS_INFO_STREAM("Camera Size" << camera_size.height << " " << camera_size.width);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pixel_to_position_transformation");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    center_point = n.advertise<visualization_msgs::Marker>("/pixel_to_position_transformation/center_point", 1);
    raw_image_sub = it.subscribe("/camera_input/image_raw", 1, get_image_size);
    D_pub = n.advertise<std_msgs::Float32>("/pixel_to_position_transformation/D", 1);

    ros::Rate r(30);
    while(ros::ok()) {
    	geometry_msgs::Point m = get_center_point(); // TODO Publish this
//    	ROS_INFO("PIXEL TO POS %lf %lf", m.x, m.y);
    	draw_camera_frame(center_point, m, camera_size);
    	draw_point(center_point, m);

    	std_msgs::Float32 dd;
    	dd.data = D;

    	D_pub.publish(dd);

    	ros::spinOnce();
    	r.sleep();
    }
}
