#pragma once

#include "Mechanism.h"
#include "Intake.h"
#include "Storage.h"
#include "Shooter.h"
#include "Feedback.h"
#include "Drive.h"
#include "Limelight.h"  
#include <frc/Timer.h>
#include <frc/Counter.h>
#include <frc/DigitalInput.h>
#include "IOMap.h"

/**PART 1: DESCRIBE MECHANISM
 * Actuators used, type of controller, and purpose:
 *     - Neo550 Motor, SparkMax - used to control the two intake bars and stage one bar (ALL ONE MOTOR)
 *     - Neo550 Motor, SparkMax - used to control the two stage two bars and the transition wheel (ALL ONE MOTOR)
 *     - Two Neo Motors, SparkMax's - used for the left and right flywheels for the shooter
 *     - "Servo" - used to move the hood forward and backwards
 *       
 * Sensors used and purpose
 *     - banner sensor - start of the balls path to count them
 *     - banner sensor - at stage one to see if there is a ball there
 *     - banner sensor - at stage two to see if there is a ball there and stop stage two
 *     - banner sensor - at shooter to count balls that leave
 *     - Encoder - internal encoders of the Neo's can be used for manual stuff probably
 *     - potentiometer - used to see where the current hood position is
 * Feedback to drivers
 *     - ball count (visual display on dashboard)
 *     - if they are ready to shoot
 * Feedback in debug menu
 *     - speed of the wheels
 *     - position of the hood
 *     - state of the intake
 *     - state of the shooter
 */

class GamEpiece : public Mechanism {
public:
    GamEpiece(Limelight* limelight);
    ~GamEpiece();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    enum PositionOnMap{
        LAUNCH_PAD,
        TARMAC_LINE,
        UP_TO_LIMELIGHT,
        UP_TO_CONTROLS
    };
    PositionOnMap positionOnMap;
    // called by controls to get the shooting wheels up to speed
    void startWarmingUpShooter(PositionOnMap positionOnMap); 

    // called by controls to say they want to start actually shooting the balls
    void startShootingTheBalls(); 
    // FOR CONTROLS PERSON: when you startShootingTheBalls() call startShootingTheBalls() as well please :D
    //warming up can be called alone, shooting should only be called with warming up aswell

    // called by controls to say they dont want to shoot 
    void stopShooting(); 

    // called by controls to start intaking balls
    void startIntaking();

    // called by controls to start outtaking balls
    void startOuttaking();

    // called by controls to have intake do nothing at all
    void doNotIntake();


    // called by controls to slowly increase the position of the hood manually  (might not be used)
    void increaseHoodPosition();

    // called by controls to slowly decrease the position of the hood manually  (might not be used)
    void decreaseHoodPosition();

    // called by controls to slowly increase the speed of the shooter manually  (might not be used)
    void increaseShooterSpeed();

    // called by controls to slowly decrease the speed of the shooter manually  (might not be used)
    void decreaseShooterSpeed();

    // used by controls for a broken switch that will turn off the ball counter
    void setBallCounterBroken(bool ballBroken);
    static const int BALL_COUNT_UNKNOWN = -1;

    
    // rework stuff for contrils
    // hood pos manual
    // stuff

private:
    Limelight* limelight;

    Intake intake {}; // not completly sure what this does but i know it's important
    Storage storage {}; // not completly sure what this does but i know it's important
    Shooter shooter {};// not completly sure what this does but i know it's important

    int currentBallCount; //how many cells we currently have in the robot.
    int cellsShot; //keeps tally of how many cells we've shot. used in the currentBallCount calculation

    bool ballCounterBroken; // used for if the sensor that counts the balls entering or leaving doesnt work

    frc::Timer brokenShotTimer; //used for timing the shot between balls if our ball counter is broken

    frc::DigitalInput intakeBeam {DIO_INTAKE_BANNER_ENTRANCE}; // first beam break on the intake
    frc::DigitalInput shooterBeam {DIO_SHOOTER_BANNER_LEFT_ROBOT}; // last beam break for shotoer

    bool lastIntakeBeamValue;

    enum IntakeState{
        INTAKE, //deploy intake mech, spin intake in, spin storage up, stop spin and retract mech when at 2 balls and comfortable in position
        OUTTAKE, //retract intake mech, spin outtake out, spin storage wheels out
        NOTTAKE //retract intake mech, stop intake spin, stop storage spin when ball is in the correct stage 1/2
    };
    // object for IntakeState setting
    IntakeState intakeState;
    
    // used by controls to set the state of the intake between INTAKE, OUTTAKE, and NOTTAKE
    void setIntakeState(IntakeState intState); 

    enum ShooterState{
        NOTSHOOTING, //default state; flywheels off, transition wheel off
        WARMUP_SHOOTER, // warmup shooter flywheels but wont shoot the balls
        WANT_TO_SHOOT, //warmup shooter flywheels, and will start to shoot when ready
        SHOOTING // when ball at top of storage, reengage stage two, wait for ball to be shot (shooterBeam triggered)
    };
    // object for ShooterState setting
    ShooterState shooterState;

    // used by controls to set the state of the shooter between WANTTOSHOOT and SHOOTING
    void setShooterState(ShooterState shootState); 
    


    // Something here...
    // hi ishan
    // hi peter
    // hi new students
    // hi jeff :D
};