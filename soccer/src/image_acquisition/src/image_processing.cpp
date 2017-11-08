/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Camera 1";

using namespace cv;

class ImageProcessing
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber raw_image_sub;
    image_transport::Publisher hsv_image_pub;
    Mat hsv;

public:
    ImageProcessing() : it(nh) {
    	raw_image_sub = it.subscribe("/camera_input/image_raw", 1, &ImageProcessing::findHSV, this);
        hsv_image_pub = it.advertise("/camera_input/image_hsv", 1);
        
        namedWindow(OPENCV_WINDOW);
    }
    ~ImageProcessing() {
        cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void findHSV(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr img;

        // Copy the input image to a pointer
        try {
            img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to the HSV
        cv::cvtColor(img->image, hsv, cv::COLOR_BGR2HSV);

        // Create and send off message
        sensor_msgs::Image hsv_img; // >> message to be sent
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, hsv);
        img_bridge.toImageMsg(hsv_img);
        hsv_image_pub.publish(hsv_img);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processing");
    ImageProcessing imageProcessing;
    ros::spin();
}
