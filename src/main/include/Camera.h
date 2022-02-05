#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <frc/DriverStation.h>
#include <units/math.h>
#include <vector>
#include <optional>

/**
 * Represents a USB Camera and handles vision processing for the cargo.
 */
class Camera : public Mechanism {
public:
    Camera();
    ~Camera();

    void process() override;

    /**
     * Returns whether a valid target was found.
     */
    bool hasTarget();

    /**
     * Returns the X location of the target in the frame (-1 to 1).
     */
    double getTargetXPosition();

    /**
     * Returns the Y location of the target in the frame (-1 to 1).
     */
    double getTargetYPosition();

    enum FrameSector {
        UNKNOWN = 0,
        LEFT,
        CENTER,
        RIGHT,
    };
    
    /**
     * Returns the sector of the frame that the target was located in.
     */
    FrameSector getTargetSector();
    
    /**
     * Returns the area of the target (Percentage of the frame 0-1).
     */
    double getTargetArea();

    /**
     * Sets whether the camera vision processing is broken.
     */
    void setVisionBroken(bool broken);
    
    /**
     * Gets whether the camera vision processing is broken.
     */
    bool getVisionBroken();
    
private:
    // The intake camera.
    cs::UsbCamera intakeCamera;
    
    // CvSink that is used to capture matricies from the camera.
    cs::CvSink cvSink;
    
    // The image stream used to send images back to dashboard.
    cs::CvSource outputStream;

    // The input image matrix.
    cv::Mat inputMatrix;

    // The output image matrix.
    cv::Mat outputMatrix;

    struct TargetData {
        // Whether the target was found.
        bool found = false;

        // X position of the target in the frame.
        double xPos = -1;

        // Y position of the target in the frame.
        double yPos = -1;

        // The area of the target in the frame.
        double area = -1;
    };
    
    // The data regarding the target.
    TargetData targetData {};

    // Vision processing broken switch.
    bool visionBroken = false;
};