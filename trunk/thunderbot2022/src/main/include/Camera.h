#pragma once

#include <Mechanism.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <frc/DriverStation.h>
#include <units/math.h>
#include <vector>

/**
 * Represents a USB Camera and handles vision processing.
 */
class Camera : public Mechanism {
public:
    Camera();
    ~Camera();

    void process() override;

    struct Frame {
        // Matrix used by OpenCV to encode images into.
        cv::Mat matrix {};
    };

    /**
     * Gets the current frame of the camera.
     */
    void getFrame(Frame* frame);

    /**
     * Sends a frame to the driver station.
     */
    void sendFrame(Frame& frame);
    
    // The region of the frame.
    enum FrameSector {
        UNKNOWN = 0,
        LEFT,
        CENTER,
        RIGHT,
    };
    
    /**
     * Attempts to locate a cargo in front of the robot and returns the sector the cargo is in.
     */
    FrameSector locateTarget(Frame& frame);
    
private:
    // The USB Camera.
    cs::UsbCamera camera;
    
    // CvSink that is used to capture matricies from the camera.
    cs::CvSink cvSink;
    
    // The image stream used to send images back to dashboard.
    cs::CvSource outputStream;
};