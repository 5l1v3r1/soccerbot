#include "camera.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
#include "constants.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;
namespace fs = boost::filesystem;

Camera::Camera() {

}

Camera::~Camera() {

}

void Camera::initialize() {
    // Loads the cascade classifiers
    string fullpath = path + ball_cascade_name;
    if (!ball_cascade.load(fullpath)) {
        printf("--(!)Error loading ball cascade\n");
        return;
    };

    // Calibrate camera
    if (CALIBRATE_CAMERA)
        calibrateCamera();
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

        // Creates intermediate stages between in and out
        process_intermediates();

        // Actual Loop through all the detections
        //detect_ball();
        // detect_field_lines();
        detect_circle();

    }
}

void Camera::process_intermediates() {
    frame_out = frame_in;
    cv::cvtColor(frame_in, frame_in_hsv, cv::COLOR_BGR2HSV);
}

void Camera::calibrateCamera() {
    // Open the camera and capture a single picture
    VideoCapture capture;
    capture.open(-1);
    if (!capture.isOpened()) {
        printf("--(!)Error opening video capture\n");
        return;
    }

    while ((char) waitKey(10) != 27); // Press ESC to capture

    capture.read(frame_in);
    if (frame_in.empty()) {
        printf(" --(!) No captured frame -- Break!");
        return;
    }
    capture.release();

    // Do the camera calibration
    Size size(9, 7);
    vector<Point2f> ptvec;
    bool found = findChessboardCorners(frame_in, size, ptvec, CALIB_CB_ADAPTIVE_THRESH);

    // Read camera parameters from XML/YAML file
    FileStorage fs(filename, FileStorage::READ);
    Mat intrinsics, distortion;
    fs["camera_matrix"] >> intrinsics;
    fs["distortion_coefficients"] >> distortion;

    // Now we are ready to find a chessboard pose using solvePnP
    vector<Point3f> boardPoints;
    // fill the array

    solvePnP(Mat(boardPoints), Mat(foundBoardCorners), cameraMatrix,
            distCoeffs, rvec, tvec, false);


}

void Camera::detect_field() {
    //cv::cvtColor(frame_in, frame_in_hsv, cv::COLOR_BGR2HSV);
}

void Camera::detect_ball() {
    std::vector<Rect> ball;
    Mat frame_gray;
    cvtColor(frame_in, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    ball_cascade.detectMultiScale(frame_gray, ball, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    frame_out = frame_in;
    for (size_t i = 0; i < ball.size(); i++) {
        Point center(ball[i].x + ball[i].width / 2, ball[i].y + ball[i].height / 2);
        ellipse(frame_out, center, Size(ball[i].width / 2, ball[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        Mat faceROI = frame_gray(ball[i]);
    }
}

void Camera::detect_circle() {

    // Construct a mask for the color "green", then perform
    // a series of dilations and erosions to remove any small
    // blobs left in the mask
    Mat mask, mask2, mask3;

    const Scalar lower = Scalar(0, 0, 190);
    const Scalar upper = Scalar(255, 100, 255);
    cv::inRange(frame_in_hsv, lower, upper, mask);


    Canny(mask, mask2, 50, 150, 3);

    int erosion_size = 1;
    Mat element = getStructuringElement(
            cv::MORPH_ELLIPSE,
            Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            Point(erosion_size, erosion_size));

    cv::dilate(mask2, mask3, element);
    // frame_out = mask3;
    // return;

    // find contours in the mask and initialize the current
    // (x, y) center of the ball
    Point center;

    std::vector<vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(mask3, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // only proceed if at least one contour was found
    std::cout << "The size of contour is : " << contours.size() << endl;
    if (contours.size() > 0) {

        // Find the largest contour in the mask        
        // For now go for the first one 
        std::vector<Point> chosen_contour;
        double max_size = 0.0;
        double temp;

        for (auto it = contours.begin(), ite = contours.end(); it != ite; ++it) {

            temp = cv::contourArea(*it, false);
            if (temp > max_size) {
                temp = max_size;
                chosen_contour = *it;
            }



            // Use it to compute the minimum enclosing circle and centroid
            float radius;
            Point2f I_dont_know;
            cv::minEnclosingCircle(chosen_contour, I_dont_know, radius);

            // Compute the center coordinates: centroids
            cv::Moments M = cv::moments(chosen_contour, false);
            center.x = (int(M.m10 / M.m00));
            center.y = (int(M.m01 / M.m00));

            // Check to ensure that the radius of the minimum enclosing
            // only proceed if the radius meets a minimum size
            std::cout << "The radius of contour is : " << radius << endl;

            if (radius > 10.0) {
                // draw the circle and centroid on the frame
                // then update the list of tracked points
                cv::circle(frame_out, I_dont_know, int(radius), Scalar(255, 0, 255), 3, 8, 0);
                cv::circle(frame_out, center, 5, Scalar(255, 255, 255), 3, 8, 0);
            }
        }
    }


}

void Camera::detect_field_lines() {
    Mat mask, mask2, mask3;
    const Scalar lower = Scalar(0, 0, 190);
    const Scalar upper = Scalar(255, 100, 255);
    cv::inRange(frame_in_hsv, lower, upper, mask);

    Canny(mask, mask2, 50, 150, 3);

    int erosion_size = 1;
    Mat element = getStructuringElement(
            cv::MORPH_ELLIPSE,
            Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            Point(erosion_size, erosion_size));

    cv::dilate(mask2, mask3, element);

    vector<Vec4i> lines;

    HoughLinesP(mask3, lines, 1, CV_PI / 180 / 4, 60, 40, 2);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(frame_out, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
    }
    //frame_out = mask3;
}

vector<string> Camera::get_image_names(string folder) {
    fs::path parent_folder(path + folder);
    fs::directory_iterator end_iter;
    vector<string> files;

    if (fs::exists(parent_folder) && fs::is_directory(parent_folder)) {
        for (fs::directory_iterator dir_iter(parent_folder); dir_iter != end_iter; ++dir_iter) {
            if (fs::is_regular_file(dir_iter->status())) {
                string pathname = dir_iter->path().filename().string();
                string extension = pathname.substr(pathname.length() - 3, pathname.length());
                if (extension.compare("jpg")) continue;
                files.push_back(pathname);
            }
        }
    }
    return files;
}

void Camera::test(string folder, void (Camera::*test_function)(void)) {
    vector<string> imgs = get_image_names(folder);
    cout << "Running tests on " << folder << endl;
    fs::path rootPath(path + folder + "test/");
    boost::system::error_code returnedError;
    fs::create_directories(rootPath, returnedError);

    for (auto it = imgs.begin(); it != imgs.end(); ++it) {
        frame_in = imread(path + folder + (*it));
        process_intermediates();
        (this->*test_function)();
        imwrite(path + folder + "test/" + (*it), frame_out);
    }
}

void Camera::run_tests() {
    test("training_images/ball/", &Camera::detect_circle);
    test("training_images/field_lines/", &Camera::detect_field_lines);
}