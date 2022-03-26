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
    Shot calculateShot(units::meter_t distance, units::meters_per_second_t yVel);

    /**
     * Calculates the angle for the drive base to be compensated by based on the
     * sideways velocity of the robot.
     */
    units::degree_t calculateAngleCompensation(units::meters_per_second_t xVel);
};