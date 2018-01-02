/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define DISPLAY_WINDOW false
#define IMAGE_ADJUST true
static const std::string OPENCV_WINDOW = "Camera 1";

using namespace cv;

class ImageProcessing
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber raw_image_sub;
    image_transport::Publisher hsv_image_pub;
    Mat colorAdjust;
    Mat hsv;

public:
    ImageProcessing() : it(nh) {
    	raw_image_sub = it.subscribe("/camera_input/image_raw", 1, &ImageProcessing::findHSV, this);
        hsv_image_pub = it.advertise("/camera_input/image_hsv", 1);
        
        if (DISPLAY_WINDOW)
        	namedWindow(OPENCV_WINDOW);
    }
    ~ImageProcessing() {
    	if (DISPLAY_WINDOW)
    		cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void findHSV(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr img;

        // Copy the input image to a pointer
        try {
            img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to the HSV
        BrightnessAndContrastAuto(img->image, colorAdjust);
        cvtColor(colorAdjust, hsv, cv::COLOR_BGR2HSV);

        // Display the image window
        if(DISPLAY_WINDOW) {
			imshow(OPENCV_WINDOW, colorAdjust);
			waitKey(100);
        }

        // Create and send off message
        sensor_msgs::Image hsv_img;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, hsv);
        img_bridge.toImageMsg(hsv_img);
        hsv_image_pub.publish(hsv_img);
    }

    /**
     *  \brief Automatic brightness and contrast optimization with optional histogram clipping
     *  \param [in]src Input image GRAY or BGR or BGRA
     *  \param [out]dst Destination image
     *  \param clipHistPercent cut wings of histogram at given percent tipical=>1, 0=>Disabled
     *  \note In case of BGRA image, we won't touch the transparency
    */
    void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent=0)
    {

        CV_Assert(clipHistPercent >= 0);
        CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

        int histSize = 256;
        float alpha, beta;
        double minGray = 0, maxGray = 0;

        //to calculate grayscale histogram
        cv::Mat gray;
        if (src.type() == CV_8UC1) gray = src;
        else if (src.type() == CV_8UC3) cvtColor(src, gray, CV_BGR2GRAY);
        else if (src.type() == CV_8UC4) cvtColor(src, gray, CV_BGRA2GRAY);
        if (clipHistPercent == 0)
        {
            // keep full available range
            cv::minMaxLoc(gray, &minGray, &maxGray);
        }
        else
        {
            cv::Mat hist; //the grayscale histogram

            float range[] = { 0, 256 };
            const float* histRange = { range };
            bool uniform = true;
            bool accumulate = false;
            calcHist(&gray, 1, 0, cv::Mat (), hist, 1, &histSize, &histRange, uniform, accumulate);

            // calculate cumulative distribution from the histogram
            std::vector<float> accumulator(histSize);
            accumulator[0] = hist.at<float>(0);
            for (int i = 1; i < histSize; i++)
            {
                accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
            }

            // locate points that cuts at required value
            float max = accumulator.back();
            clipHistPercent *= (max / 100.0); //make percent as absolute
            clipHistPercent /= 2.0; // left and right wings
            // locate left cut
            minGray = 0;
            while (accumulator[minGray] < clipHistPercent)
                minGray++;

            // locate right cut
            maxGray = histSize - 1;
            while (accumulator[maxGray] >= (max - clipHistPercent))
                maxGray--;
        }

        // current range
        float inputRange = maxGray - minGray;

        alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
        beta = -minGray * alpha;             // beta shifts current range so that minGray will go to 0

        // Apply brightness and contrast normalization
        // convertTo operates with saurate_cast
        src.convertTo(dst, -1, alpha, beta);

        // restore alpha channel from source
        if (dst.type() == CV_8UC4)
        {
            int from_to[] = { 3, 3};
            cv::mixChannels(&src, 4, &dst,1, from_to, 1);
        }
        return;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processing");
    ImageProcessing imageProcessing;
    ros::spin();
}
