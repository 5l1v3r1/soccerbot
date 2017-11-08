#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "camera.hpp"
#include "stdio.h"
using namespace std;
using namespace ros;

image_transport::Publisher field_roi;
Publisher field_coordinates;

void find_field_area(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    ros::init(argc, argv, "field_ROI");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/camera_input/image_hsv", 1, &find_field_area);
    field_roi = it.advertise("/object_recognition/field_ROI", 1);
    field_coordinates = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/field_coordinates", 1);

    ros::spin();
}
