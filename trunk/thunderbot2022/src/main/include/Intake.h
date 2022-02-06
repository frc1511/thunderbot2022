 #pragma once

#include "Mechanism.h"
#include "Feedback.h"
#include "IOMap.h"
#include "ThunderSparkMax.h"
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

/** has two sensors and two motors
 * stage one banner sensor and stage two banner sensor
 * and two double solonoids
 */
class Intake : public Mechanism
{
public:
    Intake();
    ~Intake();
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    enum IntakeDirection
    {
        INTAKE,
        OUTTAKE,
        NOTTAKE,
        MANUAL,
        SHOOTING
    };

/// Fixme:  What does SHOOTING here mean? 
    void setIntakeDirection(IntakeDirection intakeDirection);
    //ture is down, false is up, only used for manual
    void setIntakePosition(bool position); 
    // positive to intake, negative to outtake, only used for manual
    void setIntakeSpeed(double speed);
    // moves stage two to shoot the balls then moves a present ball to stage one if there is one
    void giveBallToShooter();
    // returns the ball count
    int returnBallCount();

    void setBallCounterBroken(bool ballBroken);

    bool finishedShooting();


private:
    //ball count!!!!
    void countOfBalls();

    // Something here...
    //retract piston is intake enabled now, needs to be changed in code
    frc::DoubleSolenoid rightIntake{frc::PneumaticsModuleType::CTREPCM, INTAKE_RIGHT_PIVOT_EXTEND, INTAKE_RIGHT_PIVOT_RETRACT}; 
    frc::DoubleSolenoid leftIntake{frc::PneumaticsModuleType::CTREPCM, INTAKE_LEFT_PIVOT_EXTEND, INTAKE_LEFT_PIVOT_RETRACT};
    ThunderSparkMax *intakeMotorStageOne;
    ThunderSparkMax *intakeMotorStageTwo;
    bool intakePosition; // used for manual and debug
    double intakeSpeed; // manual? i think and too lazy to check
    int ballCount; // self explanitory
    bool ballCounterBroken; // true= ball counter does not work fine    false=ball counter works fine
    bool stageOneLast; // used for ball count so you only count a ball once
    bool shooterBeamLast; // used for ball count so you only count a ball once
    bool moveBallUp; // move ball up

    IntakeDirection targetDirection;
    frc::DigitalInput stageOneFlag {DIO_STORAGE_BANNER_STAGE_ONE}; //true is present, false is not present
    frc::DigitalInput stageTwoFlag {DIO_STORAGE_BANNER_STAGE_TWO}; //true is present, false is not present
    frc::DigitalInput shooterBallCount {DIO_SHOOTER_BANNER_LEFT_ROBOT}; //true is present, false is not present

};