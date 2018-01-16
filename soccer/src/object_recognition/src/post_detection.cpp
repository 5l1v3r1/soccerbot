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
//#include <object_recognition/FieldBoundary.h>
#include <humanoid_league_msgs/LineIntersectionInImage.h>

using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

Publisher line_points_in_image;
Publisher lines_in_image;
image_transport::Subscriber hsv_img;
image_transport::Publisher post_img;
int image_count = 0;

Scalar lower = Scalar(0, 13, 125);
Scalar upper = Scalar(120, 77, 204);

Vec2f field_boundaries[4] = { 0 };
int num_boundaries = 1;

void detect_post(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Post Detection");
	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	Mat white_mask ,white_part, dilated;
	inRange(img->image, lower, upper, white_mask);
	dilate( white_mask, dilated, MORPH_RECT );
	bitwise_and(img->image,img->image,white_part,white_mask);
	
	Mat mask;
	Canny(white_part, mask, 1500, 4000, 5);

	Mat final = img->image.clone();

	vector<Vec2f> lines;
	HoughLines(mask, lines, 1, CV_PI / 180, 87, 0, 0, 15*PI/16, 17*PI/16); //100

	vector<Point2f> intersections;
//	intersections = findIntersections(lines, field_boundaries, num_boundaries);
	
	drawLinesOnImg(final, lines, Scalar(0, 255, 0));
//	drawIntersectionsOnImg(final, intersections, Scalar(255,0,0));

	saveImage(*nh, final, "lines", "final", ++image_count);
	saveImage(*nh, mask, "lines", "mask", image_count);
	saveImage(*nh, white_part, "lines", "white", image_count);

	std_msgs::Header header;
	sensor_msgs::ImagePtr post_img_msg = cv_bridge::CvImage(header, "bgr8", final).toImageMsg();
	post_img.publish(post_img_msg);

}

//void callback_field_boundary(const object_recognition::FieldBoundaryConstPtr& msg)
//{
//	num_boundaries = msg->num_lines;
//
//	for(size_t i = 0; i < num_boundaries; i++ )
//	{
//		field_boundaries[i][0] = msg->boundaries_ele1[i];
//		field_boundaries[i][1] = msg->boundaries_ele2[i];
//	}
//}

int main(int argc, char **argv) {

	ros::init(argc, argv, "post_detection");
	ros::NodeHandle n;
	nh = &n;
	
	image_transport::ImageTransport it(n);
	hsv_img = it.subscribe("/camera_input/image_hsv", 1, &detect_post);
//	ros::Subscriber field_boundary = n.subscribe("/object_recognition/field_boundary", 1, callback_field_boundary);
	line_points_in_image = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/line_points_in_image", 1);
	lines_in_image = n.advertise<humanoid_league_msgs::LineInformationInImage>("/object_recognition/lines_in_image", 1);
    post_img = it.advertise("/object_recognition/post_area", 1);


	ros::spin();
}
