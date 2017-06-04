#include "camera.hpp"

using namespace cv;
using namespace std;

Camera::Camera() {

}

Camera::~Camera() {

}

void Camera::detect_edges() {
    VideoCapture cap(CV_CAP_ANY); // open the video camera no. 0

    if (!cap.isOpened()) // if not success, exit program
        exit(1);

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); // get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    namedWindow("Camera_Input", CV_WINDOW_AUTOSIZE);

    while (1) {
        cap >> frame;
        cvtColor(frame, edges, CV_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("Camera_Input", edges);
        if (waitKey(30) < 0) break;
    }

    cvDestroyWindow("Camera_Input");
}

void Camera::detectAndDisplay(Mat frame) {
    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
    for (size_t i = 0; i < faces.size(); i++) {
        Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
        ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        Mat faceROI = frame_gray(faces[i]);
        std::vector<Rect> eyes;
        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        for (size_t j = 0; j < eyes.size(); j++) {
            Point eye_center(faces[i].x + eyes[j].x + eyes[j].width / 2, faces[i].y + eyes[j].y + eyes[j].height / 2);
            int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
            circle(frame, eye_center, radius, Scalar(255, 0, 0), 4, 8, 0);
        }
    }
    //-- Show what you got
    imshow(window_name, frame);
}

void Camera::detect_face() {
    char *path = NULL;
    path = getcwd(NULL, 0);
    if (path == NULL){
        printf("--(!)Error loading path to soccer folder");
        return;
    }
    
    rng = RNG(12345);
    VideoCapture capture;
    Mat frame;

    //-- 1. Load the cascades
    if (!face_cascade.load(path+face_cascade_name)) {
        printf("--(!)Error loading face cascade\n");
        return;
    };
    if (!eyes_cascade.load(path+eyes_cascade_name)) {
        printf("--(!)Error loading eyes cascade\n");
        return;
    };

    //-- 2. Read the video stream
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
        //-- 3. Apply the classifier to the frame
        detectAndDisplay(frame);
        int c = waitKey(10);
        if ((char) c == 27) {
            break;
        } // escape
    }
}