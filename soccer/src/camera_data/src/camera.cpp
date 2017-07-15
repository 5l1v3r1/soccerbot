#include "camera.hpp"

using namespace cv;
using namespace std;

Camera::Camera() {
    // Loads the cascade classifiers
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
    while (capture.read(frame)) {
        if (frame.empty()) {
            printf(" --(!) No captured frame -- Break!");
            break;
        }
        
        //detect_ball();
        detect_field_lines();
        
        int c = waitKey(10);
        if ((char) c == 27) {
            break;
        }
    }
}

void Camera::detect_ball() {
    std::vector<Rect> ball;
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    //-- Detect faces
    ball_cascade.detectMultiScale(frame_gray, ball, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
    for (size_t i = 0; i < ball.size(); i++) {
        Point center(ball[i].x + ball[i].width / 2, ball[i].y + ball[i].height / 2);
        ellipse(frame, center, Size(ball[i].width / 2, ball[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        Mat faceROI = frame_gray(ball[i]);
    }
    imshow(window_name, frame);
}

void Camera::detect_field_lines() {
    Mat s1, s2, s3;
    blur(frame, s1, Size(3,3) );
    Canny(s1, s2, 50, 200, 3);
    cvtColor(s2, s3, CV_GRAY2BGR);

#if 1
    vector<Vec2f> lines;
    HoughLines(s2, lines, 1, CV_PI / 180, 100, 0, 0);

    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(s3, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
    }
#else
    vector<Vec4i> lines;
    HoughLinesP(s2, lines, 1, CV_PI / 180, 50, 50, 10);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
    }
#endif
    
    imshow(window_name, s3);
}