// hi jeff

#include "ShotCalculus.h"
#include "Shooter.h"

// The height of the shooter from the ground. inches
#define SHOOTER_HEIGHT 29
// The height of the high hub from the ground. inches
#define HIGH_HUB_HEIGHT ((8*12) + 8) 
// The radius of the high hub. inches
#define HIGH_HUB_RADIUS 26
// The vertical height of the high hub itself (Agitator to the rim). inches
#define HIGH_HUB_SIZE 25

// The velocity of the cargo per 100 RPM.
#define VELOCITY_PER_100_RPM 0.4

// The angle of the hood at its minimum position.
#define HOOD_MIN_ANGLE 2
// The angle of the hood at its maximum position.
#define HOOD_MAX_ANGLE 80

/**
 * Difference in the cargo's velocity at max hood angle compared to the
 * baseline at min hood angle. Used to compensate for the position of the hood
 * affecting the exit velocity of the cargo.
 */
#define HOOD_ANGLE_VELOCITY_FACTOR 1

/**
 * A factor used to generate the impact angle of the cargo to the hub (will be
 * the impact angle when the distance is at 1 meter).
 */
#define DISTANCE_IMPACT_ANGLE_FACTOR 80

#define DISTANCE_2_IMPACT_ANGLE(distance) (std::sqrt(1/distance) * DISTANCE_IMPACT_ANGLE_FACTOR)

#define VELOCITY_GOOD_NUMBER 1
#define HOOD_ANGLE_GOOD_NUMBER 1
#define SHOOTER_RPM_GOOD_NUMBER 1

ShotCalculus::ShotCalculus() {

}

ShotCalculus::~ShotCalculus() {

}

ShotCalculus::Shot ShotCalculus::calculateShot(double distance, double velocity, double angle) {
    x1 = 0;
    y1 = 0;
    x2 = 0.0254*(distance + (HIGH_HUB_RADIUS/2)); // meters cause physics does that
    y2 = 0.0254*(HIGH_HUB_HEIGHT-SHOOTER_HEIGHT); // meters cause physics does that

    finalAngle = 45;
    
    xVelocity = 0;


    Shot shot {};

    // --- Step 9: Determine the hood position based on the desired exit angle ---


    // Get the target percentage of the hood range.
    /*double pct = ((exitAngle - HOOD_MIN_ANGLE) / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE)).value();

    // Get the full range of the hood position.
    double range = HOOD_MAX_POS - HOOD_MIN_POS;

    shot.hoodPos = pct * range + HOOD_MIN_POS;
    

    // --- Step 10: Determine the shooter RPM based on the desired exit velocity ---

    
     * VELOCITY_PER_100_RPM   exitVelocity
     * –––––––––––––––––––– = ––––––––––––
     *         100             shooterRPM
     
    shot.shooterRPM = ((100 * exitVelocity) / VELOCITY_PER_100_RPM).value();*/
    return shot;
}