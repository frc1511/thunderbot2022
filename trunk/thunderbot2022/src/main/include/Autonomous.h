#pragma once

#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Feedback.h"
#include "Camera.h"
#include "Controls.h"
#include <frc/Timer.h>

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive, GamEpiece* gamEpiece, Controls* controls);
    ~Autonomous();

    enum AutoMode {
        DO_NOTHING = 0,
    
        /**
         * Drive out of the tarmac (taxi).
         */
        UBER,

        /**
         * 1. Start facing the hub.
         * 2. Shoot cargo.
         * 3. Drive out of the tarmac.
         */
        ONE_BALL,

        /**
         * 1. Start at the left starting location.
         * 2. Pick up ball 1.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        LEFT_TWO_BALL,

        /**
         * 1. Start at the center starting location.
         * 2. Pick up ball 2.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        CENTER_TWO_BALL,

        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         */
        RIGHT_TWO_BALL,
        
        /**
         * 1. Start at the center starting location.
         * 2. Pick up ball 2.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        CENTER_THREE_BALL,

        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_SHORT_THREE_BALL,

        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_FAR_THREE_BALL,

        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Pick up ball 4.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_FOUR_BALL,

        AUTO_FOR_TREVOR_ZERO,
        AUTO_FOR_TREVOR_ONE,
        AUTO_FOR_TREVOR_TWO,
        AUTO_FOR_TREVOR_THREE,
        AUTO_FOR_TREVOR_FOUR
    };

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;


private:
    Drive* drive;
    GamEpiece* gamEpiece;
    Controls *controls;
    enum StartingPosition {
        UNKNOWN = 0,
        LEFT,
        CENTER,
        RIGHT,
    };

    StartingPosition startPosition = UNKNOWN;

    void doNothing();
    void uber();
    void oneBall();
    void leftTwoBall();
    void centerTwoBall();
    void rightTwoBall();
    void centerThreeBall();
    void rightShortThreeBall();
    void rightFarThreeBall();
    void rightFourBall();
    void autoForTrevor();

    void alignAndShoot(Shooter::ShooterMode shooterMode, unsigned ballNum);

    AutoMode currentMode = DO_NOTHING;

    frc::Timer timer {};

    bool shootingIsDone = false;

    int step = 0;
    int shootStep = 0;

    
};