#include "camera.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

using namespace cv;
using namespace std;
namespace fs = boost::filesystem;

Camera::Camera() {
    // Loads the cascade classifiers
    path = path + "soccerbot/soccer/src/camera_data/";
    string fullpath = path + ball_cascade_name;
    cout << fullpath << endl;
    if (!ball_cascade.load(fullpath)) {
        printf("--(!)Error loading ball cascade\n");
        return;
    };
}

Camera::~Camera() {

}

void Camera::loop() {
    rng = RNG(12345);
    VideoCapture capture;
    
    capture.open(-1);
    if (!capture.isOpened()) {
        printf("--(!)Error opening video capture\n");
        return;
    }
    while (capture.read(frame_in)) {
        if (frame_in.empty()) {
            printf(" --(!) No captured frame -- Break!");
            break;
        }
        
        frame_out = frame_in;
        
        //detect_ball();
        detect_field_lines();
        
        if ((char) waitKey(10) == 27)  break;
    }
}

void Camera::detect_ball() {
    std::vector<Rect> ball;
    Mat frame_gray;
    cvtColor(frame_in, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    ball_cascade.detectMultiScale(frame_gray, ball, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
    
    for (size_t i = 0; i < ball.size(); i++) {
        Point center(ball[i].x + ball[i].width / 2, ball[i].y + ball[i].height / 2);
        ellipse(frame_out, center, Size(ball[i].width / 2, ball[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        Mat faceROI = frame_gray(ball[i]);
    }
    
    imshow(camera_window, frame_in);
}

void Camera::detect_field_lines() {
    Mat s1, s2;
    blur(frame_in, s1, Size(3,3) );
    Canny(s1, s2, 100, 200, 3);
    cvtColor(s2, frame_out, CV_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(s2, lines, 1, CV_PI / 180, 50, 50, 10);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(frame_out, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
    }
    
    imshow(camera_window, frame_out);
}

vector<string> get_image_names(string folder) {
    
}

void Camera::test(string folder, void (Camera::*test_function)(void)) {
    vector<string> imgs = get_image_names(path + folder);
    for(auto it = imgs.begin(); it != imgs.end(); ++it) {
        frame_in = imread(path + folder + "/" + (*it));
        //test_function();
        imwrite(path + folder + "_test/" + (*it), frame_out);
    }
}

void Camera::run_tests() {
    test("training_images/ball/", &Camera::detect_ball);
    test("training_images/field_lines/", &Camera::detect_field_lines);
}