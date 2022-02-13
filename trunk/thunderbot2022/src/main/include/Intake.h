#pragma once

#include "Mechanism.h"
#include "Feedback.h"
#include "IOMap.h"
#include "ThunderSparkMax.h"
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <frc/Counter.h>

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
    // true is down, false is up, only used for manual
    void setIntakePosition(bool position);
    // positive to intake, negative to outtake, only used for manual
    void setIntakeSpeed(double speed);
    // moves stage two to shoot the balls then moves a present ball to stage one if there is one

    int returnBallCount();

    bool finishedShooting();

private:
    // ball count!!!!

    void switchStates();
    void ballCountIntake(bool currentSensorOneInput, bool currentSensorTwoInput);
    void ballCountOuttake(bool currentSensorOneInput, bool currentSensorTwoInput);
    bool checkSensor(frc::DigitalInput* sensor);
    enum States // States for what intake should do
    {
        STATE_INTAKE_TWO_BALL,
        STATE_INTAKE_ONE_BALL,
        
        STATE_OUTTAKE,
        STATE_STOP,
        STATE_MANUAL,
        STATE_SHOOT
    };
    // Something here...
    // retract piston is intake enabled now, needs to be changed in code                intake forward           intake in robot
    frc::DoubleSolenoid horizontalIntake{frc::PneumaticsModuleType::CTREPCM, INTAKE_PIVOT_RETRACT_PISTON, INTAKE_PIVOT_EXTEND_PISTON};
    ThunderSparkMax *intakeMotorStageOne;
    ThunderSparkMax *intakeMotorStageTwo;
    bool intakePosition; // used for manual and debug
    double intakeSpeed;  // manual? i think and too lazy to check
    int ballCount;       // self explanitory

    bool stageOneSensorPrevious; // used for ball count so you only count a ball once
    bool stageTwoSensorPrevious; // used for stage two occupied
    bool shooterSensorPrevious;  // used for ball count

    bool stageTwoOccupied; // indicates if stage 2 has a ball in it

    States currentState;
    IntakeDirection targetDirection;
    frc::DigitalInput stageOneFlag{DIO_STORAGE_BANNER_STAGE_ONE}; // true is present, false is not present
    frc::DigitalInput stageTwoFlag{DIO_STORAGE_BANNER_STAGE_TWO}; // true is present, false is not present
    frc::DigitalInput shooterFlag{DIO_SHOOTER_BANNER_LEFT_ROBOT}; // true is present, false is not present
};
