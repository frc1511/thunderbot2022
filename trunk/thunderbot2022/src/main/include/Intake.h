#pragma once

#include "Mechanism.h"
#include "Feedback.h"
#include "IOMap.h"
#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
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
        FORCE_STAGE_TWO
    };

    void setIntakeDirection(IntakeDirection intakeDirection);
    void setIntakePosition(bool position); // true is down, false is up
    void setIntakeSpeed(double speed);     // positive to intake, negative to outtake, intake must be down to work correctly

private:
    // Something here...
    frc::DoubleSolenoid rightIntake{frc::PneumaticsModuleType::CTREPCM, INTAKE_RIGHT_PIVOT_EXTEND, INTAKE_RIGHT_PIVOT_RETRACT};
    frc::DoubleSolenoid leftIntake{frc::PneumaticsModuleType::CTREPCM, INTAKE_LEFT_PIVOT_EXTEND, INTAKE_LEFT_PIVOT_RETRACT};
    rev::CANSparkMax intakeMotorStageOne{CAN_INTAKE_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax intakeMotorStageTwo{CAN_INTAKE_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    bool intakeposition;
    double intakespeed;
    IntakeDirection targetdirection;
    frc::DigitalInput stageOneFlag {DIO_STORAGE_BANNER_STAGE_ONE}; //true is present, false is not present
    frc::DigitalInput stageTwoFlag {DIO_STORAGE_BANNER_STAGE_TWO}; //true is present, false is not present

};