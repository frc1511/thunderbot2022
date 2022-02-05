#include "Camera.h"

#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

#define CAMERA_FPS 30
#define CAMERA_EXPOSURE 40

/**
 * FIXME: Determine low and high thresholds for red and blue balls.
 */

// The red cargo low threshold value.
#define THRESHOLD_CARGO_RED_LOW cv::Scalar(0, 0, 0)
// The red cargo high threshold value.
#define THRESHOLD_CARGO_RED_HIGH cv::Scalar(0, 0, 0)
// The blue cargo low threshold value.
#define THRESHOLD_CARGO_BLUE_LOW cv::Scalar(0, 0, 0)
// The blue cargo high treshold value.
#define THRESHOLD_CARGO_BLUE_HIGH cv::Scalar(0, 0, 0)

Camera::Camera() 
  : intakeCamera(frc::CameraServer::StartAutomaticCapture(USB_INTAKE_CAMERA)) {

    intakeCamera.SetResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    intakeCamera.SetFPS(CAMERA_FPS);
    intakeCamera.SetExposureManual(CAMERA_EXPOSURE);

    // Give OpenCV access to the video stream.
    cvSink = frc::CameraServer::GetVideo();
    
    // Send the video stream to the dashboard.
    outputStream = frc::CameraServer::PutVideo("intake_camera", CAMERA_WIDTH, CAMERA_HEIGHT);
}

Camera::~Camera() {
    //hi trevor :D
}

void Camera::process() {
    // Only do vision processing in the autonomous period and when the vision is
    // not broken.
    if (getCurrentMode() != MODE_AUTO || visionBroken) {
        targetData = {};
        return;
    }

    // Grab a frame from the camera and encode it into the image matrix.
    if (!cvSink.GrabFrame(inputMatrix)) {
        // Notify the output if there was an error getting the frame.
        outputStream.NotifyError(cvSink.GetError());
    }

    // Convert the color space from BGR to HSV.
    cv::cvtColor(inputMatrix, inputMatrix, cv::COLOR_BGR2HSV);

    // Apply treshold values for the cargo (depending on the alliance color).
    switch (frc::DriverStation::GetAlliance()) {
        case frc::DriverStation::kRed:
            cv::inRange(inputMatrix, THRESHOLD_CARGO_RED_LOW, THRESHOLD_CARGO_RED_HIGH, inputMatrix);
            break;
        case frc::DriverStation::kBlue:
            cv::inRange(inputMatrix, THRESHOLD_CARGO_BLUE_LOW, THRESHOLD_CARGO_BLUE_HIGH, inputMatrix);
            break;
        case frc::DriverStation::kInvalid:
            return;
    }

    // Blur the image.
    cv::medianBlur(inputMatrix, inputMatrix, 3);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Get the contours of the targets.
    cv::findContours(inputMatrix, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    // Draw the contours to the output matrix.
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Scalar contourColor(255, 0, 0);
        cv::drawContours(outputMatrix, contours, (int)i, contourColor, 2, 8, hierarchy, 0, cv::Point());
    }

    // Make sure it found some contours.
    if (contours.size() == 0) {
        targetData = {};
        return;
    }

    double largestArea = 0;
    size_t pizzo = 0;
    
    // Find the largest mass.
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > largestArea) {
            largestArea = area;
            pizzo = i;
        }
    }

    std::vector<cv::Point> targetContours = contours[pizzo];

    // Make sure the area is big enough because we want to be confident that the
    // thing the camera sees is actually the cargo. Ideally, we should never be
    // relying on this vision processing system for detecting something less
    // than a few meters away anyways.
    if (largestArea < .35) {
        targetData = {};
        return;
    }

    // Get moments.
    cv::Moments moments = cv::moments(targetContours, false);

    // Get the center of the cargo.
    cv::Point2f center = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

    // Set the properties of the target we found.
    targetData = { true, center.x, center.y, largestArea };

    // Send the output matrix to the dashboard.
    outputStream.PutFrame(outputMatrix);
}

bool Camera::hasTarget() {
    return targetData.found;
}

double Camera::getTargetXPosition() {
    // Convert to -1 to 1.
    double x = (targetData.xPos - (CAMERA_WIDTH / 2)) / (CAMERA_WIDTH / 2);
    return x;
}

double Camera::getTargetYPosition() {
    // Convert to -1 to 1.
    double y = (targetData.yPos - (CAMERA_HEIGHT / 2)) / (CAMERA_HEIGHT / 2);
    return y;
}

Camera::FrameSector Camera::getTargetSector() {
    const size_t sectorSize = CAMERA_WIDTH / 3;

    if (!targetData.found) {
        return UNKNOWN;
    }
    
    // Get sector of the frame.
    if (targetData.xPos < sectorSize) {
        return LEFT;
    }
    else if (targetData.xPos > sectorSize * 2) {
        return RIGHT;
    }
    else {
        return CENTER;
    }
}

double Camera::getTargetArea() {
    // Convert area to percentage.
    double area = targetData.area / (CAMERA_WIDTH * CAMERA_HEIGHT);
    return area;
}

void Camera::setVisionBroken(bool broken) {
    visionBroken = broken;
}

bool Camera::getVisionBroken() {
    return visionBroken;
}