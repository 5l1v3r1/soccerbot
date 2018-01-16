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


void find_game_area(const sensor_msgs::ImageConstPtr& msg) {

}

void extend_field_area(const sensor_msgs::PointCloud2Ptr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "game_GUI");
    ros::NodeHandle n;

    ros::spin();
}
