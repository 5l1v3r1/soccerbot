#include "camera.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "constants.hpp"

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
    
    if(RECALIBRATE_CAMERA)
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

        if ((char) waitKey(10) == 27) break;
    }
}

void Camera::process_intermediates() {
    frame_out = frame_in;
    cv::cvtColor(frame_in, frame_in_hsv, cv::COLOR_BGR2HSV);
}

void Camera::calibrateCamera() {
    FileStorage fs(path + calibrationFile, FileStorage::READ); // Read the settings
    cout << path + calibrationFile << endl;
    if (!fs.isOpened()) {
        cout << "Could not open the configuration file: \"" << path + calibrationFile << "\"" << endl;
        return;
    }
    fs["Settings"] >> settings;
    fs.release(); // close Settings file
    cout << "Actuttally " << endl;
    
    if (!settings.goodInput) {
        cout << "Invalid input detected. Application stopping. " << endl;
        return;
    }
    
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = settings.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
    const char ESC_KEY = 27;

    for (int i = 0;; ++i) {
        Mat view;
        bool blinkOutput = false;

        view = settings.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if (mode == CAPTURING && imagePoints.size() >= (unsigned) settings.nrFrames) {
            if (runCalibrationAndSave(settings, imageSize, cameraMatrix, distCoeffs, imagePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
        }
        if (view.empty()) // If no more images then run calibration, save and stop loop.
        {
            if (imagePoints.size() > 0)
                runCalibrationAndSave(settings, imageSize, cameraMatrix, distCoeffs, imagePoints);
            break;
        }


        imageSize = view.size(); // Format input image.
        if (settings.flipVertical) flip(view, view, 0);

        vector<Point2f> pointBuf;

        bool found;
        switch (settings.calibrationPattern) // Find feature points on the input format
        {
            case Settings::CHESSBOARD:
                found = findChessboardCorners(view, settings.boardSize, pointBuf,
                        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
                break;
            case Settings::CIRCLES_GRID:
                found = findCirclesGrid(view, settings.boardSize, pointBuf);
                break;
            case Settings::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid(view, settings.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
                break;
            default:
                found = false;
                break;
        }

        if (found) // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (settings.calibrationPattern == Settings::CHESSBOARD) {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(11, 11),
                        Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            }

            if (mode == CAPTURING && // For camera only take new samples after delay time
                    (!settings.inputCapture.isOpened() || clock() - prevTimestamp > settings.delay * 1e-3 * CLOCKS_PER_SEC)) {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = settings.inputCapture.isOpened();
            }

            // Draw the corners.
            drawChessboardCorners(view, settings.boardSize, Mat(pointBuf), found);
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

        if (mode == CAPTURING) {
            if (settings.showUndistorsed)
                msg = format("%d/%d Undist", (int) imagePoints.size(), settings.nrFrames);
            else
                msg = format("%d/%d", (int) imagePoints.size(), settings.nrFrames);
        }

        putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

        if (blinkOutput)
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if (mode == CALIBRATED && settings.showUndistorsed) {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char) waitKey(settings.inputCapture.isOpened() ? 50 : settings.delay);

        if (key == ESC_KEY)
            break;

        if (key == 'u' && mode == CALIBRATED)
            settings.showUndistorsed = !settings.showUndistorsed;

        if (settings.inputCapture.isOpened() && key == 'g') {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if (settings.inputType == Settings::IMAGE_LIST && settings.showUndistorsed) {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                imageSize, CV_16SC2, map1, map2);

        for (int i = 0; i < (int) settings.imageList.size(); i++) {
            view = imread(settings.imageList[i], 1);
            if (view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char) waitKey();
            if (c == ESC_KEY || c == 'q' || c == 'Q')
                break;
        }
    }


    return;
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