#include "Intake.h"
const double kSpeedStageOne = .7;
const double kSpeedStageTwo = .7;
Intake::Intake()
{
}

Intake::~Intake()
{
}

void Intake::resetToMode(MatchMode mode)
{
    // hi
}

void Intake::sendFeedback()
{
}

void Intake::process()
{
    switch (targetdirection)
    {
    case INTAKE:
        leftIntake.Set(frc::DoubleSolenoid::Value::kForward);
        rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        if (stageTwoFlag.Get() == false)
        {
            intakeMotorStageOne.Set(kSpeedStageOne);
            intakeMotorStageTwo.Set(kSpeedStageTwo);
        }
        else if (stageTwoFlag.Get() == true && stageOneFlag.Get() == false)
        {
            intakeMotorStageOne.Set(kSpeedStageOne);
            intakeMotorStageTwo.Set(0);
        }
        else if (stageTwoFlag.Get() == true && stageOneFlag.Get() == true)
        {
            intakeMotorStageTwo.Set(0);
            intakeMotorStageOne.Set(0);
        }

        break;
    case OUTTAKE:
        leftIntake.Set(frc::DoubleSolenoid::Value::kForward);
        rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        intakeMotorStageOne.Set(-kSpeedStageOne); // IDK what to put for reverse...so negatives
        intakeMotorStageTwo.Set(-kSpeedStageTwo);
        break;
    case NOTTAKE:
        leftIntake.Set(frc::DoubleSolenoid::Value::kOff); // IDK if I should use kReverse? I assume it is this though
        rightIntake.Set(frc::DoubleSolenoid::Value::kOff);
        intakeMotorStageOne.Set(0);
        intakeMotorStageTwo.Set(0);
        break;
    case FORCE_STAGE_TWO: // if the sensors are broken?
        leftIntake.Set(frc::DoubleSolenoid::Value::kForward);
        rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        intakeMotorStageOne.Set(kSpeedStageOne);
        intakeMotorStageTwo.Set(kSpeedStageTwo);
        break;
    }
}

void Intake::setIntakeDirection(IntakeDirection intakeDirection)
{
    targetdirection = intakeDirection;
}

void Intake::setIntakePosition(bool position)
{
    intakeposition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed)
{
    intakespeed = speed;
} // positive to intake, negative to outtake, intake must be down to work correctly
