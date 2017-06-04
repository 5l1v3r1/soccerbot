#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "camera.hpp"
using namespace std;

int main(int argc, char **argv)
{
  // Set up ROS.ca
  ros::init(argc, argv, "camera_data");
  Camera camera;
  camera.detect_face();

  ROS_INFO("Cam Tested!");
}
