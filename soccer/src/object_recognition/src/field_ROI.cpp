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
#include "camera.hpp"
#include "stdio.h"
using namespace std;
using namespace ros;
using namespace cv;

#define DISPLAY_WINDOW true
static const string WINDOW_NAME = "Field ROI";

// Publisher Subscribers
ros::NodeHandle* nh;
image_transport::Publisher field_roi;
Publisher field_coordinates;

// Constants
const Scalar lower = Scalar(45, 100, 50);
const Scalar upper = Scalar(85, 255, 200);
int image_count = 0;

void find_field_area(const sensor_msgs::ImageConstPtr& msg) {
	Mat mask, mask2, mask3;
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	inRange(img->image, lower, upper, mask);

	int erosion_size = 15;
	Mat element = getStructuringElement(
			MORPH_ELLIPSE,
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

	// We are done with the field
	if (DISPLAY_WINDOW) {
		imshow(WINDOW_NAME, img->image);
		waitKey(3);
	}

	// Save information
    bool image_test;
	nh->getParam("image_test", image_test);
	if(image_test) {
		string fileName = "../../../src/object_recognition/test/field/test" + std::to_string(++image_count) + ".png";
		ROS_ERROR("File is %s", fileName.c_str());
		try {
			imwrite(fileName, img->image);
		} catch (runtime_error& ex) {
			ROS_ERROR(ex.what());
		}
	}

	// Send the ROI to the next node
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
    field_roi = it.advertise("/object_recognition/field_ROI", 1);
    field_coordinates = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/field_coordinates", 1);

    ros::spin();
}
