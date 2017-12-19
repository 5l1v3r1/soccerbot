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

using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

Publisher line_points_in_image;
Publisher lines_in_image;
int image_count = 0;

void detect_lines(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat mask, mask2, mask3;
	const Scalar lower = Scalar(0, 0, 190);
	const Scalar upper = Scalar(255, 100, 255);
	cv::inRange(img->image, lower, upper, mask);

	Canny(mask, mask2, 50, 150, 3);

	int erosion_size = 1;
	Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));

	cv::dilate(mask2, mask3, element);

	vector<Vec4i> lines;
	Vec4i longest;
	int longestLength = 0;
	HoughLinesP(mask3, lines, 1, CV_PI / 180 / 4, 60, 40, 2);
	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];

		line(mask, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1,
				CV_AA);
	}

	bool image_test;
	nh->getParam("image_test", image_test);
	if (image_test) {
		string fileName = "../../../src/object_recognition/test/lines/test"
				+ std::to_string(++image_count) + ".png";
		ROS_INFO("File tested %s", fileName.c_str());
		try {
			imwrite(fileName, mask3);
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
	image_transport::Subscriber hsv_img = it.subscribe(
			"/object_recognition/field_ROI", 1, &detect_lines);
	line_points_in_image = n.advertise<sensor_msgs::PointCloud2>(
			"/object_recognition/line_points_in_image", 1);
	lines_in_image = n.advertise<humanoid_league_msgs::LineInformationInImage>(
			"/object_recognition/lines_in_image", 1);

	ros::spin();
}
