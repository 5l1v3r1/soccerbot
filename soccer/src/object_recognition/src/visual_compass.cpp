#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <humanoid_league_msgs/VisualCompassRotation.h>
#include <iostream>
#include "camera.hpp"
#include "stdio.h"
using namespace std;
using namespace ros;

image_transport::Subscriber hsv_img;
Publisher vc_rotation;

void find_compass(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    ros::init(argc, argv, "visual_compass");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    hsv_img = it.subscribe("/camera_input/image_hsv", 1, &find_compass);
    vc_rotation = n.advertise<humanoid_league_msgs::VisualCompassRotation>("/object_recognition/vc_rotation", 1);

    ros::spin();
}
