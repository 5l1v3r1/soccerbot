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

image_transport::Publisher game_roi;
image_transport::Subscriber hsv_img;
Subscriber field_coordinates;

void find_game_area(const sensor_msgs::ImageConstPtr& msg) {

}

void extend_field_area(const sensor_msgs::PointCloud2Ptr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "game_ROI");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    hsv_img = it.subscribe("/camera_input/image_hsv", 1, &find_game_area);
    field_coordinates = n.subscribe("/object_recognition/field_coordinates", 1, &extend_field_area);
    game_roi = it.advertise("/object_recognition/game_ROI", 1);

    ros::spin();
}
