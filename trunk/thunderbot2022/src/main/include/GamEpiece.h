#pragma once

// RIP "#pragma thrice", Forever in our hearts, Revision 74 to Revision 107

#include "Mechanism.h"
#include "Intake.h"
#include "Shooter.h"
#include "Feedback.h"
#include "Drive.h"
#include "Limelight.h"  
#include <frc/Timer.h>
#include <frc/Counter.h>
#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
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
    
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    
    // Enables or disables shooter warm up for the specified location
    void setShooterWarmUpEnabled(Shooter::ShooterMode shooterMode, bool enabled); 

    // Begins shooting a ball using the specified location/aiming mode.  If warm-up is enabled,
    // the shooterMode here is ignored and the one from the warmup is used; 
    // the shot cannot be cancelled/aborted once it has begun.
    // This will only shoot one ball and if this is called when a shot is in progress already,
    // the new shot request is ignored.
    void shootABall(Shooter::ShooterMode shooterMode);

    // Indicates if a shot is in progress; true means one is
    bool isShotInProgress();

    // Used to control the hood motor manually.  Only used when a shot or warm up is in progress
    // Values are -1 -> 1 with negative numbers turning to retract the hood and positive extending it.
    //  (used only for manual operation, otherwise ignored)
    void setManualHoodSpeed(double hoodSpeed);





    enum IntakeDirection{
        INTAKE, //deploy intake mech, spin intake in, spin storage up, stop spin and retract mech when at 2 balls and comfortable in position
        OUTTAKE, // intake pivot remains where it is, spin outtake out, spin storage wheels out
        NOTTAKE, //retract intake mech, stop intake spin, stop storage spin when ball is in the correct stage 1/2
        MANUAL // NOT USED BY CONTROLS used for manual :D
    };

    void setIntakeDirection(IntakeDirection intDir);

    // called to set the speed of the intake and storage manually (used only for manual operation, otherwise ignored)
    // -1 to 1, negative numbers spin to eject balls, positive ones intake balls
    void setManualIntakeSpeed(double intakeSpeed);
    // called to set the positioon of the intake manually (used only for manual operation, otherwise ignored)
    // true is deployed and false is retracted/stored
    void setManualIntakePosition(bool intakePosition);


    // obtains the current ball count from intake :D
    int getCurrentBallCount();

    // checks if a ball is at stage one
    bool ballAtStageOne();

    // changes the manual speed by 100, great for testing, true will increase, false will decrease.
    void changeShooterSpeed(bool increaseOrDecrease);

    // cancels the process of shooting, use when something goes wrong and it tries to shoot whne it shouldnt, has absolute control, very powerful
    void cancelShot();


private:
    Limelight* limelight;

    Intake intake {}; // not completly sure what this does but i know it's important
    Shooter shooter { limelight };// not completly sure what this does but i know it's important

    int currentBallCount; //how many cells we currently have in the robot.
    int cellsShot; //keeps tally of how many cells we've shot. used in the currentBallCount calculation

    bool ballCounterBroken; // used for if the sensor that counts the balls entering or leaving doesnt work

    bool ballJustShot;
    bool waitingForTimer = false;
    frc::Timer shotTimer; //used for timing the shot between balls if our ball counter is broken


    
    bool beforeShotCount;
    
    

    enum ShooterState{
        NOT_SHOOTING, //default state; flywheels off, transition wheel off
        WARMUP_SHOOTER, // warmup shooter flywheels but wont shoot the balls
        WANT_TO_SHOOT, //warmup shooter flywheels, and will start to shoot when ready
        SHOOTING // when ball at top of storage, reengage stage two, wait for ball to be shot (shooterBeam triggered)
    };
    // object for ShooterState setting
    ShooterState shooterState;
    ShooterState desiredShooterState;

    
    IntakeDirection intakeDirection;
    


    // Something here...
    // hi ishan
    // hi peter
    // hi new students
    // hi jeff :D
};