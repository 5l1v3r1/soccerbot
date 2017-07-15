#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iostream>

using namespace cv;
using namespace std;

class Camera {
private:
    Mat frame;
    Mat edges;

    /** Global variables */
    string ball_cascade_name = "soccerbot/soccer/src/camera_data/cascades/ball.xml";
    CascadeClassifier ball_cascade;
    string window_name = "Capture - Ball detection";
    RNG rng;

    void detect_ball(Mat frame);
public:
    Camera();
    ~Camera();
    std::string path;

    void loop();
    
    void detect_ball();
    void detect_field_lines();
};

#endif /* CAMERA_INPUT_HPP */

