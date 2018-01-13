#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace ros;

image_transport::Subscriber field_img;
image_transport::Publisher optical_flow;

Mat img, flow, frame, original;
UMat flowUmat, prevgray;
ros::NodeHandle* nh;

void detect_depth(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImageConstPtr imgptr;
	try {
		imgptr = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// save original for later
	imgptr->image.copyTo(original);

	// just make current frame gray
	cvtColor(imgptr->image, img, COLOR_BGR2GRAY);

	if (prevgray.empty() == false) {

		// calculate optical flow
		calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);

		// copy Umat container to standard Mat
		flowUmat.copyTo(flow);

		// By y += 5, x += 5 you can specify the grid
		for (int y = 0; y < original.rows; y += 5) {
			for (int x = 0; x < original.cols; x += 5) {
				// get the flow from y, x position * 10 for better visibility
				const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
				// draw line at flow direction
				line(original, Point(x, y),
						Point(cvRound(x + flowatxy.x),
								cvRound(y + flowatxy.y)),
						Scalar(255, 0, 0));
				// draw initial point
				circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);

			}

		}

		// Publish the optical flow
		std_msgs::Header header;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", original).toImageMsg();
		optical_flow.publish(msg);

		// fill previous image again
		img.copyTo(prevgray);

	} else {

		// fill previous image in case prevgray.empty() == true
		img.copyTo(prevgray);

	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "visual_odom");
	ros::NodeHandle n;
	nh = &n;

	image_transport::ImageTransport it(n);
	field_img = it.subscribe("/camera_input/image_raw", 1, &detect_depth);
	optical_flow = it.advertise("/object_recognition/optical_flow", 1);

	ros::spin();
}
