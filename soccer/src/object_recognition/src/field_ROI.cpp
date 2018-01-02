#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Polygon.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include "camera.hpp"
#include "stdio.h"
#include "../include/statistics/kde.hpp"
#include <std_msgs/Int32.h>
#include <image_acquisition/colorspace.h>

using namespace std;
using namespace ros;
using namespace cv;

static const string WINDOW_NAME = "Field ROI";

// Publisher Subscribers
ros::NodeHandle* nh;
image_transport::Publisher field_roi;
image_transport::Subscriber hsv_img;
Publisher field_coordinates;

// Constants
Scalar lower = Scalar(45, 100, 50);
Scalar upper = Scalar(85, 255, 200);

int image_count = 0;

bool sortbyangle(Vec2f a, Vec2f b) {
	return a[1] < b[1];
}

void find_field_area(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat mask, mask2, mask3;
	inRange(img->image, lower, upper, mask);

	int erosion_size = 5;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));

	// Dilate and Erode for small parts
	dilate(mask, mask2, element);
	erode(mask2, mask3, element);

	bitwise_not(mask3, mask3);
	dilate(mask3, mask3, element);
	erode(mask3, mask3, element);
	bitwise_not(mask3, mask3);

	double minarea = ((double) (640 * 480) / 30);
	double tmparea = 0.0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(mask3, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); i++) {
		tmparea = contourArea(contours[i]);
		if (tmparea < minarea) {
			drawContours(mask3, contours, i, CV_RGB(0, 0, 0), CV_FILLED);
		}
	}
	bitwise_not(mask3, mask3);
	findContours(mask3, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); i++) {
		tmparea = contourArea(contours[i]);
		if (tmparea < minarea) {
			drawContours(mask3, contours, i, CV_RGB(0, 0, 0), CV_FILLED);
		}
	}
	bitwise_not(mask3, mask3);

	dilate(mask3, mask3, element);
	erode(mask3, mask3, element);

	// Find the polygon information
	Mat edges, cedges;
	Canny(mask3, edges, 50, 200, 3);
	cvtColor(edges, cedges, CV_GRAY2BGR);

	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI / 180, 50, 0, 0);

	// Go through the vertical lines and find ones that are not straight
	sort(lines.begin(), lines.end(), sortbyangle);
	KDE kde;
	for (auto it = lines.begin(); it != lines.end(); ++it) {
		kde.add_data((*it)[1]);
	}
	vector<double> pdf;
	kde.pdf(pdf);

	// Hill climb to find peaks
	vector<double> peaks;
	for(auto it = pdf.begin(); it != pdf.end(); ++it) {
		//TODO finish this part
	}



	Mat imtest;
	cvtColor(img->image, imtest, cv::COLOR_HSV2BGR);
	for (size_t i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(cedges, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	}

	// Save information
	bool image_test;
	nh->getParam("image_test", image_test);
	if (image_test) {
		string fileName = "../../../src/object_recognition/test/field/test"
				+ std::to_string(++image_count) + ".png";
		ROS_INFO("File  %s", fileName.c_str());
		try {
			imwrite(fileName, cedges);
		} catch (runtime_error& ex) {
			ROS_ERROR(ex.what());
		}
	}

}

void callback_colorspace(const image_acquisition::colorspace::ConstPtr& msg)
{
	//update colorspace
	lower = Scalar(msg->lower_hue, 100, 50);
	upper = Scalar(msg->upper_hue, 255, 200);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "field_ROI");
	ros::NodeHandle n;
	nh = &n;

#ifdef DISPLAY_WINDOW
	namedWindow(WINDOW_NAME);
#endif

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/camera_input/image_hsv", 1, &find_field_area);
    ros::Subscriber colorspace = n.subscribe("/image_acquisition/colorspace", 1, callback_colorspace);
    field_roi = it.advertise("/object_recognition/field_ROI", 1);
    field_coordinates = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/field_coordinates", 1);

	ros::spin();
}