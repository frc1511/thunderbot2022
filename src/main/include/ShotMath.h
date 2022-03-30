#pragma once

#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>
#include <cmath>

/**
 * A utility class for generating trajectories from anywhere on the field. Also
 * compensates for the robot's current velocity to shoot while driving.
 */
class ShotMath {
public:
    ShotMath();
    ~ShotMath();
    
    void Feedback();

    struct Shot {
        double hoodPos = 0;
        double shooterRPM = 0;
    };

    /**
     * Calculates the hood position and shooter RPM based on the distance of the
     * robot from the high hub and the forwards/backwards velocity of the robot.
     * Will only work consistently if the robot is aligned with the hub,
     * perferably through the vice grip mechanism.
     */
    Shot calculateShot(units::meter_t distance, units::meters_per_second_t yVel, units::degree_t theAngle);

    /**
     * Calculates the angle for the drive base to be compensated by based on the
     * sideways velocity of the robot.
     */
    units::degree_t calculateAngleCompensation(units::meters_per_second_t xVel);
    
private:
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    units::meter_t xVertexDistance = 0_m;
    units::meters_per_second_t xVelocity = 0_mps;
    units::meter_t yVertexDistance = 0_m;
    units::meters_per_second_t yVelocity = 0_mps;
    units::second_t vertexTime = 0_s;
    units::degree_t impactAngle = 0_deg;
    double a = 0;
    double b = 0;
    double c = 0;
    double m = 0;
    units::degree_t hoodAngle = 0_deg;
    units::meters_per_second_t shooterVelocity = 0_mps;
    double hoodPosition = 0;
    double shooterRPM = 0;
    double velocityY = 0;
    double velocityX = 0;
    double velocity = 0;
    double angle = 0;
};