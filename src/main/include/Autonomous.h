#pragma once

#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Feedback.h"
#include "Camera.h"
#include <frc/Timer.h>

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive, GamEpiece* gamEpiece, Camera* camera);
    ~Autonomous();

    enum AutoMode {
        DO_NOTHING = 0,
    
        /**
         * 1. Align with high hub.
         * 2. Shoot ball.
         */
        ONE_BALL,
    
        /**
         * 1. Pick up ball behind the robot.
         * 2. Align with high hub.
         * 3. Shoot both balls.
         */    
        LEFT_TWO_BALL,
        CENTER_TWO_BALL,
        RIGHT_TWO_BALL,
        
        /**
         * 1. Pick up ball behind the robot.
         * 2. Align with high hub.
         * 3. Shoot both balls.
         * 4. Pick up ball at the alliance station wall.
         * 5. Align with high hub.
         * 6. Shoot ball.
         */
        LEFT_THREE_BALL,
        CENTER_THREE_BALL,
        RIGHT_THREE_BALL,

        /**
         * (Start in center spot).
         *
         * 1. Pick up ball behind the robot.
         * 2. Align with high hub.
         * 3. Shoot both balls.
         * 4. Pick up ball at the alliance station wall.
         * 5. Align with high hub.
         * 6. Shoot ball.
         * 7. Pick up two balls at cargo line.
         * 8. Align with high hub.
         * 9. Shoot both balls.
         */
        FIVE_BALL,
    };

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
    void doNothing();
    void oneBall();
    void leftTwoBall();
    void centerTwoBall();
    void rightTwoBall();
    void leftThreeBall();
    void centerThreeBall();
    void rightThreeBall();
    void fiveBall();

    bool rotateToCargo();

    AutoMode currentMode = DO_NOTHING;

    frc::Timer timer {};
    int step = 0;

    Drive* drive;
    GamEpiece* gamEpiece;

    Camera* camera;
    Camera::Frame frame {};
};