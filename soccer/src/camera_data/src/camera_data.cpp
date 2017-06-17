#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "camera.hpp"
#include "stdio.h"
using namespace std;

int main(int argc, char **argv) {
    // Set up ROS.ca
    ros::init(argc, argv, "camera_data");
    Camera camera;
    std::string path(argv[0]);
    int idx = path.find("devel/lib/camera_data/camera_data");
    path = path.substr(0, idx - 1);
    camera.path = path;
    camera.detect_ball();

    ROS_INFO("Cam Tested!");
}
