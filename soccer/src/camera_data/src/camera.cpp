#include "camera.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

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
        detect_field_lines();

        if ((char) waitKey(10) == 27) break;
    }
}

void Camera::process_intermediates() {
    frame_out = frame_in;
    cv::cvtColor(frame_in, frame_in_hsv, cv::COLOR_BGR2HSV);
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
    Mat s1;
    cvtColor(frame_in, s1, cv::COLOR_RGB2HSV);
//    cv::inRange
//    mask = cv2.inRange(hsv, greenLower, greenUpper)
//    mask = cv2.erode(mask, None, iterations=2)
//    mask = cv2.dilate(mask, None, iterations=2)
//    
//    # find contours in the mask and initialize the current
//    # (x, y) center of the ball
//    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
//                            cv2.CHAIN_APPROX_SIMPLE)[-2]
//    center = None
//
//    # only proceed if at least one contour was found
//    if len(cnts) > 0:
//        # find the largest contour in the mask, then use
//        # it to compute the minimum enclosing circle and
//        # centroid
//        c = max(cnts, key=cv2.contourArea)
//        ((x, y), radius) = cv2.minEnclosingCircle(c)
//        M = cv2.moments(c)
//        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
//
//        # only proceed if the radius meets a minimum size
//        if radius > 10:
//            # draw the circle and centroid on the frame,
//            # then update the list of tracked points
//            cv2.circle(frame, (int(x), int(y)), int(radius),
//                       (0, 255, 255), 2)
//            cv2.circle(frame, center, 5, (0, 0, 255), -1)
//
//    # update the points queue
//    pts.appendleft(center)
//
//    # loop over the set of tracked points
//    for i in xrange(1, len(pts)):
//        # if either of the tracked points are None, ignore
//        # them
//        if pts[i - 1] is None or pts[i] is None:
//            continue
//
//        # otherwise, compute the thickness of the line and
//        # draw the connecting lines
//        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
//        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
}

void Camera::detect_field_lines() {
    Mat mask, mask2, mask3;
    const Scalar lower = Scalar(0,0,190);
    const Scalar upper = Scalar(255,100,255);
    cv::inRange(frame_in_hsv, lower, upper, mask);
    
    Canny(mask, mask2, 50, 150, 3);
    
    int erosion_size = 1;
    Mat element = getStructuringElement(
            cv::MORPH_ELLIPSE,
            Size(2*erosion_size+1, 2*erosion_size+1),
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
    fs::path rootPath (path + folder + "test/");
    boost::system::error_code returnedError;
    fs::create_directories( rootPath, returnedError );
    
    for (auto it = imgs.begin(); it != imgs.end(); ++it) {
        frame_in = imread(path + folder + (*it));
        process_intermediates();
        (this->*test_function)();
        imwrite(path + folder + "test/" + (*it), frame_out);
    }
}

void Camera::run_tests() {
    test("training_images/ball/", &Camera::detect_ball);
    test("training_images/field_lines/", &Camera::detect_field_lines);
}