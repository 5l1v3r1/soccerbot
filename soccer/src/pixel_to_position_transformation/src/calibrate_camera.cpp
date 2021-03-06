#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"
#include "../include/settings.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

    ros::init(argc, argv, "calibrate_camera");
    ros::NodeHandle n;

    Settings settings;
    string calibrationFile = "../../../src/object_recognition/configuration/in_VID5.xml";

    FileStorage fs(calibrationFile, FileStorage::READ); // Read the settings

    if (!fs.isOpened()) {
        cout << "Could not open the configuration file: \"" << calibrationFile << "\"" << endl;
        return EXIT_FAILURE;
    }
    fs["Settings"] >> settings;
    //cout << settings.aspectRatio << endl;
    fs.release(); // close Settings file

    if (!settings.goodInput) {
        cout << "Invalid input detected. Application stopping. " << endl;
        return EXIT_FAILURE;
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
            ROS_ERROR("FOUND");

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
            return EXIT_SUCCESS;
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

}
