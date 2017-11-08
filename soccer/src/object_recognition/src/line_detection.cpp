#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <humanoid_league_msgs/LineInformationInImage.h>

using namespace std;
using namespace ros;

Publisher line_points_in_image;
Publisher lines_in_image;

void detect_lines(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    ros::init(argc, argv, "line_detection");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/object_recognition/field_ROI", 1, &detect_lines);
    line_points_in_image = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/line_points_in_image", 1);
    lines_in_image = n.advertise<humanoid_league_msgs::LineInformationInImage>("/object_recognition/lines_in_image", 1);

    ros::spin();
}
