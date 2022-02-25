#pragma once

#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Feedback.h"
#include "Camera.h"
#include <frc/Timer.h>

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive, GamEpiece* gamEpiece);
    ~Autonomous();

    enum AutoMode {
        DO_NOTHING = 0,
    
        /**
         * Drive out of the tarmac (taxi).
         */
        UBER,

        /**
         * 1. Start in the left starting location.
         * 2. Align with high hub.
         * 3. Shoot cargo.
         */
        LEFT_ONE_BALL,

        /**
         * 1. Start in the center starting location.
         * 2. Align with high hub.
         * 3. Shoot cargo.
         */
        CENTER_ONE_BALL,

        /**
         * 1. Start in the right starting location.
         * 2. Align with high hub.
         * 3. Shoot cargo.
         */
        RIGHT_ONE_BALL,

        /**
         * 1. Start in the left starting location.
         * 2. Pick up ball 1.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        LEFT_TWO_BALL,

        /**
         * 1. Start in the center starting location.
         * 2. Pick up ball 2.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        CENTER_TWO_BALL,

        /**
         * 1. Start in the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        RIGHT_TWO_BALL,
        
        /**
         * 1. Start in the center starting location.
         * 2. Pick up ball 2.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        CENTER_THREE_BALL,

        /**
         * 1. Start in the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_SHORT_THREE_BALL,

        /**
         * 1. Start in the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_FAR_THREE_BALL,

        /**
         * 1. Start in the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_FOUR_BALL,
    };

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
    enum StartingPosition {
        UNKNOWN = 0,
        LEFT,
        CENTER,
        RIGHT,
    };

    StartingPosition startPosition = UNKNOWN;

    void doNothing();
    void uber();
    void leftOneBall();
    void centerOneBall();
    void rightOneBall();
    void leftTwoBall();
    void centerTwoBall();
    void rightTwoBall();
    void centerThreeBall();
    void rightShortThreeBall();
    void rightFarThreeBall();
    void rightFourBall();

    bool alignAndShoot(Shooter::ShooterMode shooterMode, unsigned ballNum);

    AutoMode currentMode = DO_NOTHING;

    frc::Timer timer {};

    int step = 0;
    int shootStep = 0;

    Drive* drive;
    GamEpiece* gamEpiece;
};