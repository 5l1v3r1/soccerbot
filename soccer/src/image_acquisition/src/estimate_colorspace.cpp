
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int32.h> 
#include <image_acquisition/SoccerColorSpace.h>

using namespace std;
using namespace cv;

ros::Publisher pub;

//hard code the range
#define HUE_RANGE 20
#define NUM_HUE_RANGES 360/HUE_RANGE
const int SAT_LOW = 95;
const int SAT_HIGH = 255;
const int VAL_LOW = 60;
const int VAL_HIGH = 200;

//test flag
#define TEST 0

int32_t num_whitedots[NUM_HUE_RANGES] = { 0 };


static int32_t img_masking(const Mat& img_hsv_in, int32_t hue_low, int32_t hue_high, int num)
{
	int32_t num_whitedot = 0;
	Scalar lower = Scalar(hue_low, SAT_LOW, VAL_LOW);
	Scalar higher = Scalar(hue_high, SAT_HIGH, VAL_HIGH);
	
	//mask input image with defined hue range
	Mat img_masked;
	inRange(img_hsv_in, lower,higher, img_masked);
	
	//count white pixels
	num_whitedot = countNonZero(img_masked);
	
exit:
	return num_whitedot;
}

static int index_most_whitedots(const Mat& img_in )
{
	int index = 0;
	int32_t lower = 0;
	int32_t higher = 0;
	int32_t max_whitedot = 0;
	int32_t tmp_whitedot = 0;
	
	
	for( int i = 0 ; i < NUM_HUE_RANGES ; i++ )
	{	
		//set lower and higher bounds
		lower = HUE_RANGE * i;
		higher = lower + HUE_RANGE - 1;
				
		tmp_whitedot = img_masking(img_in,lower,higher,i);
		
		//update the index with most whitedots
		if( max_whitedot < tmp_whitedot)
		{
			max_whitedot = tmp_whitedot;
			index = i;
		}
	}
	
	return index;
}

int callbackCount;

static void callback_getImage(const sensor_msgs::ImageConstPtr& msg)
{
	if (callbackCount++ % 10) return; // Call back only every 10 images
	
	cv_bridge::CvImageConstPtr cv_ptr = NULL;
	image_acquisition::SoccerColorSpace msg_send;
	int index = 0;
	int32_t low_hue = 0;
	int32_t high_hue = 0;
	
	try
    {
		// retrieve the received img
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		if( NULL == cv_ptr )
			goto exit;
		
		//explore the color range of field
		index = index_most_whitedots(cv_ptr->image);
		low_hue = HUE_RANGE * index;
		high_hue = low_hue + HUE_RANGE - 1;
		
		//publish the values as msg
		msg_send.grass.upper_hue = high_hue;
		msg_send.grass.lower_hue = low_hue;
		msg_send.grass.upper_sat = SAT_HIGH;
		msg_send.grass.lower_sat = SAT_LOW;
		msg_send.grass.upper_val = VAL_HIGH;
		msg_send.grass.lower_val  = VAL_LOW;
		pub.publish(msg_send);
		
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      goto exit;
    }
	
exit:
	return;
}

int main(int argc, char **argv) 
{
	//ros setup
	ros::init(argc, argv, "estimate_colorspace");
    ROS_INFO("Estimate Colorspace");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
	//subscribe same topic as field_ROI node
    callbackCount = 0;
	image_transport::Subscriber sub = it.subscribe("/camera_input/image_hsv", 1, &callback_getImage);
	pub = n.advertise<image_acquisition::SoccerColorSpace>("/image_acquisition/colorspace", 1);

	if(TEST)
	{
		Mat img_in,masked_img,img_hsv;
		int index = 0;
		int32_t low_hue = 0;
		int32_t high_hue = 0;
		image_acquisition::SoccerColorSpace msg_send;
		ros::Rate rate(5);
		
		img_in = imread("/soccerbot/soccer/src/image_acquisition/images/field/5.jpg",CV_LOAD_IMAGE_COLOR);   //change to abs path
		cvtColor(img_in,img_hsv,COLOR_BGR2HSV);
		
		//explore the color range of field
		index = index_most_whitedots(img_hsv);
		low_hue = HUE_RANGE * index;
		high_hue = low_hue + HUE_RANGE - 1;
		
		//define the lower/upper value of the range -> masking
		Scalar lower = Scalar(low_hue, SAT_LOW, VAL_LOW);
		Scalar higher = Scalar(high_hue, SAT_HIGH, VAL_HIGH); 
		inRange(img_hsv, lower, higher, masked_img);
		
		//save the masked img
		imwrite("/soccerbot/soccer/src/image_acquisition/images/field/test/5_tes_2.jpg",masked_img);     //change to abs path
		
		for(int i =0; i < 5 ; i++ )
		{
			//publish the values as msg
			msg_send.grass.upper_hue = high_hue;
			msg_send.grass.lower_hue = low_hue;
			pub.publish(msg_send);
			cout << "sent" << i << ":" << msg_send.grass.upper_hue << "," << msg_send.grass.lower_hue << endl;
			
			rate.sleep();

		}
	}
	
	ros::spin();
}
