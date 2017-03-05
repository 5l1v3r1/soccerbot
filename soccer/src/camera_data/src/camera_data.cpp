#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class cam_test{
public:
    cam_test() {
        VideoCapture cap(CV_CAP_ANY);		// open the video camera no. 0

	if (!cap.isOpened())			// if not success, exit program
	    exit(1);		

        double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 	// get the width of frames of the video
        double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);	// get the height of frames of the video
        cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	Mat edges;	
        namedWindow("Camera_Input",CV_WINDOW_AUTOSIZE);
	
        while (1)
        {
	    Mat frame;
	    cap >> frame;	   
	    cvtColor(frame, edges, CV_BGR2GRAY);
	    GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	    Canny(edges, edges, 0 , 30, 3);
	    imshow("Camera_Input", edges);
	    if (waitKey(30) > 0) break;
        }
    }

    ~cam_test(){
        cvDestroyWindow("Camera_Input"); //Destroy Window
    }

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "camera_data");
  cam_test cam_object;

  ROS_INFO("Cam Tested!");
}
