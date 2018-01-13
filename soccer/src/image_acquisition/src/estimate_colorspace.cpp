
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
#define HUE_CHANGE_SCALE 10	
#define HUE_GREREN_MAX 200
#define NUM_HUE_RANGES HUE_GREREN_MAX/HUE_CHANGE_SCALE

#define SAT_RANGE_LINE 25
#define SAT_CHANGE_SCALE 12
#define SAT_MAX 72
#define NUM_SAT_RANGES	SAT_MAX/SAT_CHANGE_SCALE

//const sat/val value for field 
const int32_t SAT_LOW = 75;
const int32_t SAT_HIGH = 255;
const int32_t VAL_LOW = 0;
const int32_t VAL_HIGH = 200;

//const sat/val value for line
const int32_t HUE_LOW_LINE = 0;
const int32_t HUE_HIGH_LINE = 360;
const int32_t VAL_LOW_LINE = 25;
const int32_t VAL_HIGH_LINE = 255;

//test flag
#define TEST 0

//int32_t num_whitedots[NUM_HUE_RANGES] = { 0 };

static int32_t img_masking(
	const Mat& img_hsv_in,
	int32_t hue_low, 
	int32_t hue_high,
	int32_t sat_low,
	int32_t sat_high,
	int32_t val_low,
	int32_t val_high
)
{
	int32_t num_whitedot = 0;
	Scalar lower = Scalar(hue_low, sat_low, val_low);
	Scalar higher = Scalar(hue_high, sat_high, val_high);
	
	//mask input image with defined hue range
	Mat img_masked;
	inRange(img_hsv_in, lower,higher, img_masked);
	
	//count white pixels
	num_whitedot = countNonZero(img_masked);
	
exit:
	return num_whitedot;
}

static void index_most_whitedots(const Mat& img_in, int *index, int *index_line )
{
	int32_t lower = 0;
	int32_t higher = 0;
	int32_t max_whitedot = 0;
	int32_t max_whitedot_line = 0;
	int32_t tmp_whitedot = 0;
	
	if( NULL == index || NULL == index_line )
		goto exit;
	
	//field
	for( int i = 0 ; i < NUM_HUE_RANGES ; i++ )
	{	
		//set lower and higher bounds
		lower = HUE_CHANGE_SCALE * i;
		higher = lower + HUE_RANGE - 1;
				
		tmp_whitedot = img_masking(img_in,lower,higher,SAT_LOW,SAT_HIGH,VAL_LOW,VAL_HIGH);
		
		//update the index with most whitedots
		if( max_whitedot < tmp_whitedot)
		{
			max_whitedot = tmp_whitedot;
			*index = i;
		}
		
	}
	
	//line
	for(int i = 0; i < NUM_SAT_RANGES; i++ )
	{
		//set lower and higher bounds
		lower = SAT_CHANGE_SCALE * i;
		higher = lower + SAT_RANGE_LINE - 1;
		
		tmp_whitedot = img_masking(img_in,HUE_LOW_LINE,HUE_HIGH_LINE,lower,higher,VAL_LOW_LINE,VAL_HIGH_LINE);
		
		//update the index with most whitedots
		if( max_whitedot_line < tmp_whitedot)
		{
			max_whitedot_line = tmp_whitedot;
			*index_line = i;
		}
		
	}
	
exit:
	return;
}

int callbackCount;

static void callback_getImage(const sensor_msgs::ImageConstPtr& msg)
{
	if (callbackCount++ % 10) return; // Call back only every 10 images
	
	cv_bridge::CvImageConstPtr cv_ptr = NULL;
	image_acquisition::SoccerColorSpace msg_send;
	int index = 0;
	int index_line = 0;
	int32_t low_hue = 0;
	int32_t high_hue = 0;
	int32_t low_sat = 0;
	int32_t high_sat = 0;
	
	try
    {
		// retrieve the received img
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		if( NULL == cv_ptr )
			goto exit;
		
		//explore the color range of field
		index_most_whitedots(cv_ptr->image, &index, &index_line);
		low_hue = HUE_RANGE * index;
		high_hue = low_hue + HUE_RANGE - 1;
		low_sat = SAT_RANGE_LINE * index_line;
		high_sat = low_sat + SAT_RANGE_LINE - 1;
		
		
		//publish the values as msg
		//field
		msg_send.grass.upper_hue = high_hue;
		msg_send.grass.lower_hue = low_hue;
		msg_send.grass.upper_sat = SAT_HIGH;
		msg_send.grass.lower_sat = SAT_LOW;
		msg_send.grass.upper_val = VAL_HIGH;
		msg_send.grass.lower_val = VAL_LOW;
		
		//line
		msg_send.field_lines.upper_hue = HUE_HIGH_LINE;
		msg_send.field_lines.lower_hue = HUE_LOW_LINE;
		msg_send.field_lines.upper_sat = high_sat;
		msg_send.field_lines.lower_sat = low_sat;
		msg_send.field_lines.upper_val = VAL_HIGH_LINE;
		msg_send.field_lines.lower_val = VAL_LOW_LINE;
		
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
		Mat img_in,masked_img,masked_img2,img_hsv;
		int index = 0;
		int index_line = 0;
		int32_t low_hue = 0;
		int32_t high_hue = 0;
		int32_t low_sat = 0;
		int32_t high_sat = 0;
		image_acquisition::SoccerColorSpace msg_send;
		ros::Rate rate(5);
		
		img_in = imread("/soccerbot/soccer/src/image_acquisition/images/field/aufnahme11_FullPic.jpg",CV_LOAD_IMAGE_COLOR);   //change to abs path
		cvtColor(img_in,img_hsv,COLOR_BGR2HSV);
		
		//explore the color range of field
		index_most_whitedots(img_hsv,&index,&index_line);
		low_hue = HUE_CHANGE_SCALE * index;
		high_hue = low_hue + HUE_RANGE - 1;
		
		//define the lower/upper value of the range -> masking
		Scalar lower = Scalar(low_hue, SAT_LOW, VAL_LOW);
		Scalar higher = Scalar(high_hue, SAT_HIGH, VAL_HIGH); 
		inRange(img_hsv, lower, higher, masked_img);
		imwrite("/soccerbot/soccer/src/image_acquisition/images/field/test/5_tes_2.jpg",masked_img);     //change to abs path
		
		//define the lower/upper value of the range -> masking
		low_sat = SAT_CHANGE_SCALE * index_line;
		high_sat = low_sat + SAT_RANGE_LINE - 1;
		lower = Scalar(HUE_LOW_LINE, low_sat, VAL_LOW_LINE);
		higher = Scalar(HUE_HIGH_LINE, high_sat, VAL_HIGH_LINE); 
		inRange(img_hsv, lower, higher, masked_img2);
		imwrite("/soccerbot/soccer/src/image_acquisition/images/field/test/5_tes_3.jpg",masked_img2);     //change to abs path
		
		
		
		for(int i =0; i < 5 ; i++ )
		{
			//publish the values as msg
			msg_send.grass.upper_hue = high_hue;
			msg_send.grass.lower_hue = low_hue;
			msg_send.field_lines.lower_sat = low_sat;
			msg_send.field_lines.upper_sat = high_sat;
			
			pub.publish(msg_send);
			cout << "hue high: " << msg_send.grass.upper_hue << " hue low: " << msg_send.grass.lower_hue << " sat low: " << msg_send.field_lines.lower_sat << " sat high: " << msg_send.field_lines.upper_sat << endl;
			
			rate.sleep();

		}
	}
	
	ros::spin();
}
