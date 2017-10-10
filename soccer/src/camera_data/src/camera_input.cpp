#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "camera.hpp"
#include "stdio.h"
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_input");
    ros::NodeHandle n;
    cout << "THING THING THINHG" << endl;
    image_transport::ImageTransport it(n);
    image_transport::CameraPublisher pub = it.advertiseCamera("/camera_input/image_raw", 1);

    std::string streamProvider = "0";
    cv::VideoCapture cap;
    n.getParam("video_stream_provider", streamProvider);

    cap.open(-1);
    if (!cap.isOpened()) {
        ROS_ERROR_STREAM("Could not open the stream.");
        return -1;
    }
    ROS_INFO_STREAM("Opened Camera, Starting to publish");

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info;
    std_msgs::Header header;
    header.frame_id = 1;

    ros::Rate r(30);
    while (n.ok()) {
        cap >> frame;
        if (pub.getNumSubscribers() > 0) {
            if (!frame.empty()) {
                msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                pub.publish(*msg, cam_info, ros::Time::now());
            }
        }
        r.sleep();
    }
}
