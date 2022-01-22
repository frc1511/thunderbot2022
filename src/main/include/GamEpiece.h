#pragma once

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

class GamEpiece {
public:
    GamEpiece(Limelight* limelight);
    ~GamEpiece();

    void process();
    void reset(int ballCount);
    void debug(Feedback* feedback);

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
        WANTTOSHOOT, //warmup shooter flywheels, wait for them to be at speed
        SHOOTING //when ball at stop of storage, reengage stage two, wait for ball to be shot (shooterBeam triggered)
    };
    // object for ShooterState setting
    ShooterState shooterState;

    // used by controls to set the state of the shooter between WANTTOSHOOT and SHOOTING
    void setShooterState(ShooterState shootState); 

    // used by controls for a broken switch that will turn off the ball counter
    void setBallCounterBroken(bool ballBroken);
    static const int BALL_COUNT_UNKNOWN = -1;



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

    bool lastIntakeBeamValue;


    // Something here...
    // hi ishan
    // hi peter
    // hi new students
};