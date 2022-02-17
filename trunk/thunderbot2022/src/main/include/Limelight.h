#pragma once

#include "Mechanism.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>
#include <memory>

/**
 * Represents the limelight sensor on the robot. :D
 */
class Limelight : public Mechanism {
public:
    Limelight();
    ~Limelight();

    /**
     * Returns whether the limelight has any valid targets.
     */
    bool hasTarget();

    /**
     * Returns the horizontal angle to the target.
     */
    units::radian_t getAngleHorizontal();

    double getDistance();

    /**
     * Returns the vertical angle to the target.
     */
    units::radian_t getAngleVertical();

    enum class LEDMode {
        DEFAULT = 0,
        OFF     = 1,
        BLINK   = 2,
        ON      = 3,
    };

    /**
     * Sets the mode of the limelight LEDs.
     */
    void setLEDMode(LEDMode mode);

    /**
     * Returns the current LED mode.
     */
    LEDMode getLEDMode();

    enum class CameraMode {
        VISION_PROCESS = 0,
        DRIVER_CAMERA  = 1,
    };

    /**
     * Sets the mode of the limelight camera.
     */
    void setCameraMode(CameraMode mode);

    /**
     * Returns the current camera mode.
     */
    CameraMode getCameraMode();

private:
    // A network table used to communicate with limelight.
    std::shared_ptr<nt::NetworkTable> table;
};