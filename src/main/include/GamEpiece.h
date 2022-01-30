#pragma once

#pragma thrice

#include "Mechanism.h"
#include "Intake.h"
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
 *     - Two Double Solonoids - used to lower and raise intake
 *       
 * Sensors used and purpose
 *     - banner sensor - start of the balls path to count them
 *     - banner sensor - at stage one to see if there is a ball there
 *     - banner sensor - at stage two to see if there is a ball there and stop stage two
 *     - banner sensor - at shooter to count balls that leave
 *     - Encoder - internal encoders of the Neo's can be used to see if shooter is at speed and for manual stuff probably also 
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
        ODOMETRY, 
        LAUNCH_PAD,
        TARMAC_LINE,// limelight decides settings
        MAUNUAL // used manual setting that were given previously
    };
    PositionOnMap positionOnMap;
    // called by controls to get the shooting wheels up to speed
    void startWarmingUpShooter(Shooter::ShooterMode shooterMode); 

    // called by controls to say they want to start actually shooting the balls
    void startShootingTheBalls(Shooter::ShooterMode shooterMode); 

    // called by controls to say they dont want to shoot 
    void stopShooting(); 

    enum IntakeDirection{
        INTAKE, //deploy intake mech, spin intake in, spin storage up, stop spin and retract mech when at 2 balls and comfortable in position
        OUTTAKE, //retract intake mech, spin outtake out, spin storage wheels out
        NOTTAKE, //retract intake mech, stop intake spin, stop storage spin when ball is in the correct stage 1/2
        MANUAL // NOT USED BY CONTROLS used for manual :D
    };

    void setIntakeDirection(IntakeDirection intDir);

    // called by controls to set the speed of the intake manually (used only for manual operation, otherwise ignored)
    void setManualIntakeSpeed(double intakeSpeed);
    // called by controls to set the positioon of the intake manually (used only for manual operation, otherwise ignored)
    void setManualIntakePosition(bool intakePosition);


    // called by controls to change the position of the hood manually  (used only for manual operation, otherwise ignored)
    void setManualHoodSpeed(double hoodSpeed);


    // (unused) called by controls to slowly increase the speed of the shooter manually  (used only for manual operation, otherwise ignored)
    // void setManualShooterSpeed(double shooterSpeed);


    // used by controls for a broken switch that will turn off the ball counter
    void setBallCounterBroken(bool ballBroken);
    static const int BALL_COUNT_UNKNOWN = -1;
    // do the motors actuators etc list
    //add priming chang shooter enumand funcrion to be more make sense
    // can add manual mode to find field position from controls and the query thing
    // hood position control stuff

    
    // rework stuff for contrils
    // hood pos manual
    // stuff

private:
    Limelight* limelight;

    Intake intake {}; // not completly sure what this does but i know it's important
    Shooter shooter { limelight };// not completly sure what this does but i know it's important

    int currentBallCount; //how many cells we currently have in the robot.
    int cellsShot; //keeps tally of how many cells we've shot. used in the currentBallCount calculation

    bool ballCounterBroken; // used for if the sensor that counts the balls entering or leaving doesnt work

    bool ballJustShot;

    frc::Timer brokenShotTimer; //used for timing the shot between balls if our ball counter is broken

    frc::DigitalInput intakeBeam {DIO_INTAKE_BANNER_ENTRANCE}; // first beam break on the intake
    frc::DigitalInput shooterBeam {DIO_SHOOTER_BANNER_LEFT_ROBOT}; // last beam break for shotoer

    bool beforeShotCount;

    

    enum ShooterState{
        NOT_SHOOTING, //default state; flywheels off, transition wheel off
        WARMUP_SHOOTER, // warmup shooter flywheels but wont shoot the balls
        WANT_TO_SHOOT, //warmup shooter flywheels, and will start to shoot when ready
        SHOOTING // when ball at top of storage, reengage stage two, wait for ball to be shot (shooterBeam triggered)
    };
    // object for ShooterState setting
    ShooterState shooterState;


    
    IntakeDirection intakeDirection;
    


    // Something here...
    // hi ishan
    // hi peter
    // hi new students
    // hi jeff :D
};