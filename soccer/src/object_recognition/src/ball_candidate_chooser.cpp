#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <humanoid_league_msgs/BallInImage.h>
#include <iostream>
#include "camera.hpp"
#include "stdio.h"
using namespace std;
using namespace ros;

Publisher ball;

void findBall(const sensor_msgs::ImageConstPtr& msg) {


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "ball_candidate_chooser");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/object_recognition/ball_candidates", 1, &findBall);
    ball = n.advertise<humanoid_league_msgs::BallInImage>("/object_recognition/ball_in_image", 1);

    ros::spin();
}
