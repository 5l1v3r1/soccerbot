#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;
using namespace ros;

image_transport::Subscriber hsv_img;
Publisher vc_rotation;

void find_compass(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    ros::init(argc, argv, "visual_odom");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    hsv_img = it.subscribe("/camera_input/image_hsv", 1, &find_compass);
    vc_rotation = n.advertise<nav_msgs::Odometry>("/object_recognition/visual_odom", 1);

    ros::spin();
}
