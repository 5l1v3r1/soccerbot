#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <humanoid_league_msgs/LineInformationInImage.h>
#include <vectormath.hpp>

using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

image_transport::Publisher game_img;
humanoid_league_msgs::LineInformationInImage field_lines;
humanoid_league_msgs::LineInformationInImage post_lines;

static void draw_lines_in_img(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Game GUI");

	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat final;
	final = img->image.clone();


	//draw lines on img
	drawLinesOnImgCartesian(final, field_lines, Scalar(255,0,0));
	drawLinesOnImgCartesian(final, post_lines, Scalar(0,255,0));

	//send out the img with lines
	std_msgs::Header header;
	sensor_msgs::ImagePtr game_img_msg = cv_bridge::CvImage(header, "bgr8", final).toImageMsg();
	game_img.publish(game_img_msg);

}

static void update_field_lines(const humanoid_league_msgs::LineInformationInImagePtr& msg_lines) {
	field_lines = *msg_lines;
}

static void update_post_lines(const humanoid_league_msgs::LineInformationInImagePtr& msg_goalnet) {
	post_lines = *msg_goalnet;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "game_GUI");
    ros::NodeHandle n;
    nh = &n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/camera_input/image_raw", 1, draw_lines_in_img);
    Subscriber lines = n.subscribe("/object_recognition/lines_in_image", 1, update_field_lines);
    Subscriber goal_posts = n.subscribe("/object_recognition/post_in_image", 1, update_post_lines);
    game_img = it.advertise("/object_recognition/game_GUI", 1);

    ros::spin();
}
