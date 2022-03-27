// hi jeff

#include "ShotMath.h"
#include "Shooter.h"

// The height of the shooter from the ground.
#define SHOOTER_HEIGHT 2.5_ft
// The height of the high hub from the ground.
#define HIGH_HUB_HEIGHT (8_ft + 8_in)
// The radius of the high hub.
#define HIGH_HUB_RADIUS 26_in
// The vertical height of the high hub itself (Agitator to the rim).
#define HIGH_HUB_SIZE 25_in

// The velocity of the cargo per 100 RPM.
#define VELOCITY_PER_100_RPM 0.4_mps

// The angle of the hood at its minimum position.
#define HOOD_MIN_ANGLE 2_deg
// The angle of the hood at its maximum position.
#define HOOD_MAX_ANGLE 80_deg

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

/**
 * Generate an impact angle based on the distance the cargo is being shot from.
 * A steeper angle will be generated when closer to the hub, and a shallower
 * angle will be generated when farther away.
 */
#define DISTANCE_2_IMPACT_ANGLE(distance) units::degree_t(std::sqrt(1/units::meter_t(distance).value()) * DISTANCE_IMPACT_ANGLE_FACTOR)

/**
 * Good numbers to allow for tuning to the trajectory when it inevitably has
 * problems. D:
 */
#define VELOCITY_GOOD_NUMBER 1
#define HOOD_ANGLE_GOOD_NUMBER 1
#define SHOOTER_RPM_GOOD_NUMBER 1

ShotMath::ShotMath() {

}

ShotMath::~ShotMath() {

}

ShotMath::Shot ShotMath::calculateShot(units::meter_t distance, units::meters_per_second_t yVel) {
    x1 = 0;
    y1 = units::meter_t(SHOOTER_HEIGHT).value();
    x2 = units::meter_t(distance + (HIGH_HUB_RADIUS/2)).value();
    y2 = units::meter_t(HIGH_HUB_HEIGHT).value();
    
    if(x1){
        // hi
    }
    /**
     * Because the distance is calculated from the edge of the high hub,
     * subtract the radius of the hub to get the distance to the center.
     */
    
    // --- Step 1: Determine the impact angle of the cargo to the high hub ---
    
    impactAngle = DISTANCE_2_IMPACT_ANGLE(x2);

    // --- Step 2: Determine the equation of the parabola using the initial point and the tangent line at the impact point ---
    
    {
        // --- Step 2.1: Find the derivative of the parabola ---

        // Get the slope of the tangent line.
        m = (-units::math::tan(impactAngle)).value();
        
        /**
         * y' = 2ax + b
         * 
         * derivative:
         * m = 2a(x2) + b
         */
        
        // --- Step 2.2: Find c ---
        /**
         * y1 = a(x1)^2 + b(x1) + c
         *
         * y1 = a(0)^2 + b(0) + c
         * 
         * y1 = c
         */
        c = y1;

        // --- Step 2.3: Find a ---
        /**
         * Use quadratic equation with target point for x and y.
         * y2 = a(x2)^2 + b(x2) + c
         * y2 = a(x2)^2 +b(x2) + c
         * Use derivarive with target point and slope for x and y'.
         * m = 2a(x2) + b
         * m(x2) = 2a(x2)^2 = b(x2)
         * 
         * Subtract equations to eliminate b.
         * x2 * (m = 2a(x2) + b) = (m(x2) = 2a(x2)^2 + b(x2))
         * 
         *   m(x2) = 2a(x2)^2 + b(x2) + 0c
         * +  (-y2   =  -a(x2)^2 - b(x2) - c)
         * ––––––––––––––––––––––––––––––––
         * m(x2) - y2 = a(x2)^2 - c
         * 
         * a = (m(x2) - y2 + c) / (x2)^2
         */
        a = ((m * x2) - y2 + c) / (x2 * x2);

        // --- Step 2.4: Find b ---
        /**
         * y2 = a(x2)^2 + b(x2) + c
         * 
         * b = (y2 - a(x2)^2 - c) / x2
         */
        b = (y2 - (a * (x2 * x2)) - c) / x2;
    }

    // --- Step 3: Using the derivative, determine the the horizontal distance to the vertex of the parabola ---

    /**
     * y' = 2ax + b
     * 
     * 0 = 2ax + b
     * 
     * x = -b / 2a
     */
    xVertexDistance = units::meter_t((-b) / (2 * a));

    // --- Step 3.5: Find the vertical distance to the vertex of the parabola ---

    /**
     * y = ax^2 + bx + c
     * 
     * dvy = a(dvx)^2 + b(dvx) + c
     */
    yVertexDistance = units::meter_t(a * (xVertexDistance.value() * xVertexDistance.value()) + (b * xVertexDistance.value()) + c);

    // --- Step 4: Determine the initial vertical velocity of the cargo ---

    /**
     * Vf^2 = Vi^2 + 2ad
     * 
     * 0 = Vi^2 + 2(-9.81)(dvy)
     * 
     * Vyi = √(-2(-9.81)(dvy))
     */
    yVelocity = units::meters_per_second_t(std::sqrt(-2 * -9.81 * (yVertexDistance.value()) - y1));

    // --- Step 5: Determine time to the vertex ---

    /**
     * v = d/t
     * 
     * vi = dvy/t
     * 
     * t = dvy/vyi
     */
    vertexTime = units::second_t((yVertexDistance.value() - y1) / yVelocity.value());

    // --- Step 6: Determine the initial horizontal velocity of the cargo ---

    /**
     * v = d/t
     * 
     * vxi = dvx/tv
     */
    xVelocity = units::meters_per_second_t(xVertexDistance.value() / vertexTime.value());

    // --- Step 6.5: Adjust the horizontal velocity of the cargo based on the robot's velocity ---

    /**
     * Subtract the robot's velocity from the cargo's horizontal velocity
     * because the velocity of the robot will be directly translated to the
     * velocity of the cargo.
     */
    // yVel is correct, peter calls it yVel for no reason, but yVel is the x velocity. D:
    xVelocity -= yVel;

    // --- Step 7: Determine the exit angle of the cargo from the shooter ---

    /**
     * θ = arctan(vy / vx)
     */
    hoodAngle = units::math::atan(yVelocity / xVelocity);

    hoodAngle *= HOOD_ANGLE_GOOD_NUMBER;

    // --- Step 8: Determine the exit velocity of the cargo from the shooter ---

    /**
     * a^2 + b^2 = c^2
     */
    shooterVelocity = units::math::hypot(xVelocity, yVelocity);

    shooterVelocity *= VELOCITY_GOOD_NUMBER;

    Shot shot {};

    // --- Step 9: Determine the hood position based on the desired exit angle ---


    // Get the target percentage of the hood range.
    double pct = ((hoodAngle - HOOD_MIN_ANGLE) / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE)).value();

    // Get the full range of the hood position.
    double range = HOOD_MAX_POS - HOOD_MIN_POS;

    shot.hoodPos = pct * range + HOOD_MIN_POS;
    

    // --- Step 10: Determine the shooter RPM based on the desired exit velocity ---

    /**
     * VELOCITY_PER_100_RPM   shooterVelocity
     * –––––––––––––––––––– = ––––––––––––
     *         100             shooterRPM
     */
    shot.shooterRPM = ((100 * shooterVelocity) / VELOCITY_PER_100_RPM).value();

    //shot.shooterRPM = std::clamp(shot.shooterRPM, 0.0, (double)SHOOTER_MAX_RPM);

    return shot;

    // Air resistance!! :D

    
}

units::degree_t ShotMath::calculateAngleCompensation(units::meters_per_second_t xVel) {
    // TODO: Do this.

    return 0_deg;
}

void ShotMath::Feedback(){
    Feedback::sendDouble("a shotMath", "x1", x1);
    Feedback::sendDouble("a shotMath", "y1", y1);
    Feedback::sendDouble("a shotMath", "x2", x2);
    Feedback::sendDouble("a shotMath", "y2", y2);
    Feedback::sendDouble("a shotMath", "x distance", xVertexDistance.value());
    Feedback::sendDouble("a shotMath", "y distance", yVertexDistance.value());
    Feedback::sendDouble("a shotMath", "x velocity", xVelocity.value());
    Feedback::sendDouble("a shotMath", "y velocity", yVelocity.value());
    Feedback::sendDouble("a shotMath", "time", vertexTime.value());
    Feedback::sendDouble("a shotMath", "impact angle", impactAngle.value());
    Feedback::sendDouble("a shotMath", "a", a);
    Feedback::sendDouble("a shotMath", "b", b);
    Feedback::sendDouble("a shotMath", "c", c);
    Feedback::sendDouble("a shotMath", "m", m);
    Feedback::sendDouble("a shotMath", "hood angle", hoodAngle.value());
    Feedback::sendDouble("a shotMath", "shooter velocity", shooterVelocity.value());
    Feedback::sendDouble("a shotMath", "hood position", hoodPosition);
    Feedback::sendDouble("a shotMath", "shooter RPM", shooterRPM);
}

// bye jeff