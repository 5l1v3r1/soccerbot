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
#include "vectormath.hpp"

using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

Publisher line_points_in_image;
Publisher lines_in_image;
image_transport::Subscriber field_img;
int image_count = 0;

Scalar lower = Scalar(0, 0, 165);
Scalar upper = Scalar(255, 105, 255);

void detect_lines(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Line Area");

	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat mask;
	Canny(img->image, mask, 80, 250, 3);

	Mat final = img->image.clone();

	vector<Vec2f> lines;
	HoughLines(mask, lines, 1, CV_PI / 180, 160, 0, 0);

	vector<Vec2f> fieldlines = filterUnparallelRepeats(lines);

	drawLinesOnImg(final, fieldlines, Scalar(255,0,0));
//	drawLinesOnImg(final, lines, Scalar(0,255,0));

	// Extract the intersections of the line
	vector<Point2f> intersections;
	for(int i = 0; i < fieldlines.size(); ++i) {
		for(int j = 0; j < fieldlines.size(); ++j) {
			if(abs(fieldlines[i][1] - fieldlines[j][1]) < CV_PI / 24) continue;

			if(i == j) continue;
			Point2f intersect = intersection(fieldlines[i], fieldlines[j]);
			intersections.push_back(intersect);
			circle(final, intersect, 5, Scalar(0,255,0));
		}
	}

	// Save information
	bool image_test;
	nh->getParam("image_test", image_test);
	if (image_test) {
		string fileName = "../../../src/object_recognition/test/lines/test"
				+ std::to_string(++image_count) + ".png";
		string fileNameOriginal = "../../../src/object_recognition/test/lines/"
				+ std::to_string(image_count) + ".png";
		try {
			imwrite(fileName, final);
			imwrite(fileNameOriginal, img->image);
		} catch (runtime_error& ex) {
			ROS_ERROR(ex.what());
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "line_detection");
	ros::NodeHandle n;
	nh = &n;

	image_transport::ImageTransport it(n);
	field_img = it.subscribe("/object_recognition/field_area", 1, &detect_lines);

	line_points_in_image = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/line_points_in_image", 1);
	lines_in_image = n.advertise<humanoid_league_msgs::LineInformationInImage>("/object_recognition/lines_in_image", 1);

	ros::spin();
}
