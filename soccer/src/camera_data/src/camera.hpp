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
    String face_cascade_name = "/home/vuwij/soccerbot/soccer/src/camera_data/src/haarcascade_frontalface_alt.xml";
    String eyes_cascade_name = "/home/vuwij/soccerbot/soccer/src/camera_data/src/haarcascade_eye_tree_eyeglasses.xml";
    CascadeClassifier face_cascade;
    CascadeClassifier eyes_cascade;
    string window_name = "Capture - Face detection";
    RNG rng;

    void detectAndDisplay(Mat frame);
public:
    Camera();
    ~Camera();

    void detect_edges();
    void detect_face();
};

#endif /* CAMERA_INPUT_HPP */

