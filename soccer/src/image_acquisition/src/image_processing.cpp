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

#define TEST_INPUT true

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;

class ImageProcessing
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber raw_image_sub;
    image_transport::Publisher raw_image_pub;
    image_transport::Publisher hsv_image_pub;
public:
    ImageProcessing() : it(nh) {
        raw_image_pub = it.advertise("/image_converter/output_video", 1);
        
        if(TEST_INPUT)
            raw_image_sub = it.subscribe("/camera_input/image_raw", 1, &ImageProcessing::imageCb, this);
        
        namedWindow(OPENCV_WINDOW);
    }
    ~ImageProcessing() {
        cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // Draw an example circle
        if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));
        
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
        
        raw_image_pub.publish(cv_ptr->toImageMsg());
    }
    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processing");
    ImageProcessing imageProcessing;
    ros::spin();
}