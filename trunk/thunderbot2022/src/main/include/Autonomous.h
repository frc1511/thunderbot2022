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
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         */
        RIGHT_THREE_BALL,

        RIGHT_FOUR_BALL,
        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         * 8. Pick up ball 4 and 5.
         * 9. Align with high hub.
         * 10. Shoot both cargo.
         */
        RIGHT_FIVE_BALL,

        /**
         * 1. Start at the right starting location.
         * 2. Pick up ball 3.
         * 3. Align with high hub.
         * 4. Shoot both cargo.
         * 5. Pick up ball 2.
         * 6. Align with high hub.
         * 7. Shoot cargo.
         * 8. Pick up ball 4 and 5.
         * 9. Go to ball 1.
         * 9. Align with high hub.
         * 10. Shoot both cargo.
         * 11. Pick up ball 1.
         * 12. Align with high hub.
         * 13. Shoot cargo.
         */
        RIGHT_SIX_BALL,

        AUTO_FOR_TREVOR_ZERO,
        AUTO_FOR_TREVOR_ONE,
        AUTO_FOR_TREVOR_TWO,
        AUTO_FOR_TREVOR_THREE,
        AUTO_FOR_TREVOR_FOUR,
        AUTO_FOR_TREVOR_FIVE,
        AUTO_FOR_TREVOR_SIX,
        AUTO_FOR_TREVOR_SEVEN,
        AUTO_FOR_TREVOR_EIGHT,
        AUTO_FOR_TREVOR_NINE,
        
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
    void rightThreeBall();
    void rightFiveBall();
    void rightSixBall();
    void autoForTrevor();

    void alignAndShoot(Shooter::ShooterMode shooterMode);

    AutoMode currentMode = DO_NOTHING;

    frc::Timer delayTimer {};
    frc::Timer autoTimer {};

    bool shootingIsDone = false;

    int step = 0;
    int shootStep = 0;
    bool autoDone = false;

    
};