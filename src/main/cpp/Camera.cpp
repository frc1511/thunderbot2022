#include "Camera.h"

#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240
#define CAMERA_FPS 30
#define CAMERA_EXPOSURE 40

// TODO Determine low and high thresholds for red and blue balls.

const cv::Scalar CAMERA_RED_LOW(0, 0, 0);
const cv::Scalar CAMERA_RED_HIGH(0, 0, 0);
const cv::Scalar CAMERA_BLUE_LOW(0, 0, 0);
const cv::Scalar CAMERA_BLUE_HIGH(0, 0, 0);

Camera::Camera()
  : camera(frc::CameraServer::StartAutomaticCapture()) {

    camera.SetResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    camera.SetFPS(CAMERA_FPS);
    camera.SetExposureManual(CAMERA_EXPOSURE);

    cvSink = frc::CameraServer::GetVideo();

    outputStream = frc::CameraServer::PutVideo("intake_camera", CAMERA_WIDTH, CAMERA_HEIGHT);
}

Camera::~Camera() {
    //hi trevor :D
}

void Camera::process() {
    static Frame frame {};
    
    getFrame(&frame);
    sendFrame(frame);
}

void Camera::getFrame(Frame* frame) {
    // Grab a frame from the camera and encode it into the image matrix.
    // Notify the output if there was an error getting the frame.
    if (!cvSink.GrabFrame(frame->matrix)) {
        outputStream.NotifyError(cvSink.GetError());
        return;
    }
}

void Camera::sendFrame(Frame& frame) {
    // Send the frame to the driver station.
    outputStream.PutFrame(frame.matrix);
}

Camera::FrameSector Camera::locateTarget(Frame& frame) {
    cv::Mat targetMatrix;

    // Apply threshold to matrix (depending on alliance color).
    switch (frc::DriverStation::GetAlliance()) {
        case frc::DriverStation::kRed:
            cv::inRange(targetMatrix, CAMERA_RED_LOW, CAMERA_RED_HIGH, targetMatrix);
            break;
        case frc::DriverStation::kBlue:
            cv::inRange(targetMatrix, CAMERA_BLUE_LOW, CAMERA_BLUE_HIGH, targetMatrix);
            break;
        case frc::DriverStation::kInvalid:
            return UNKNOWN;
    }
    
    // Blur image.
    cv::medianBlur(targetMatrix, targetMatrix, 3);    
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    
    // Get the contours of the target.
    cv::findContours(targetMatrix, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    const size_t contourNum = contours.size();
    
    if (contourNum == 0) {
        return UNKNOWN;
    }
    
    // Draw contours on image.
    for (size_t i = 0; i < contourNum; ++i) {
        cv::Scalar contourColor(255, 0, 0);
        cv::drawContours(targetMatrix, contours, (int)i, contourColor, 2, 8, hierarchy, 0, cv::Point());
    }
    
    double largestArea = 0;
    std::vector<cv::Point>* targetContours;
    
    // Find the largest mass.
    for (size_t i = 0; i < contourNum; ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > largestArea) {
            largestArea = area;
            targetContours = &contours[i];
        }
    }

    // TODO Check to make sure area is above a certain value.

    // Get moments.
    cv::Moments moments = cv::moments(*targetContours, false);
    
    // Get the center.
    cv::Point2f center = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
    
    const size_t sectorSize = CAMERA_WIDTH / 3;
    
    // Get sector of the frame.
    if (center.x < sectorSize) {
        return LEFT;
    }
    else if (center.x > sectorSize * 2) {
        return RIGHT;
    }
    else {
        return CENTER;
    }
}