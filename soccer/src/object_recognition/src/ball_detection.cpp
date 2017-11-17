#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace ros;

image_transport::Publisher ball_candidates;

void findBall(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    init(argc, argv, "ball_detection");
    NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/object_recognition/field_ROI", 1, &findBall);
    ball_candidates = it.advertise("/object_recognition/ball_candidates", 1);

    ros::spin();
}
