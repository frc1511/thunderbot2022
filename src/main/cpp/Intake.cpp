#include "Intake.h"
const double kSpeedStageOne = .7;
const double kSpeedStageTwo = .7;
const double kSpeedStageTwoSlow = .3;
Intake::Intake()
{
}

Intake::~Intake(){

}

void Intake::resetToMode(MatchMode mode)
{
    targetDirection = NOTTAKE;
    ballCount = 0;
}



void Intake::process()
{
    // adds to ball count
    if(stageOneFlag.Get() == true){ // there is a ball in front
            stageOneLast = true;
        }
    if (stageOneFlag.Get() == false && stageOneLast == true){ // there is not a ball in front but it left
        // Do not change cell count away from UNKNOWN (-1)
        if (ballCount != -1){
            ballCount++;
        }
        stageOneLast = false;
    }
    // takes away from ball count
    if(shooterBallCount.Get() == true){ // there is a ball in front
            shooterBeamLast = true;
        }
    if (shooterBallCount.Get() == false && shooterBeamLast == true){ // there is not a ball in front but it left
        // Do not change cell count away from UNKNOWN (-1)
        if (ballCount != -1){
            ballCount+= -1;
        }
        shooterBeamLast = false;
    }

    // if the ball count is a number it shouldnt be then make it the unknown value
    if(ballCount < 0 || ballCount > 2){
        ballCount = -1;
    }

    switch (targetDirection)
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
        leftIntake.Set(frc::DoubleSolenoid::Value::kReverse); // IDK if I should use kReverse? I assume it is this though. yes
        rightIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        intakeMotorStageOne.Set(0);
        intakeMotorStageTwo.Set(0);
        break;
    case MANUAL:
        if(intakePosition){
            leftIntake.Set(frc::DoubleSolenoid::Value::kForward);
            rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else{
            leftIntake.Set(frc::DoubleSolenoid::Value::kReverse);
            rightIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        }
        intakeMotorStageOne.Set(intakeSpeed);
        intakeMotorStageTwo.Set(intakeSpeed);
        break;
    case SHOOTING:
        // exists so that it doesnt control stage 2 to do bad things
        break;
    }
}

void Intake::setIntakeDirection(IntakeDirection intakeDirection)
{
    targetDirection = intakeDirection;
}

void Intake::setIntakePosition(bool position)
{
    intakePosition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed)
{
    intakeSpeed = speed;
} // positive to intake, negative to outtake

void Intake::giveBallToShooter(){
    //put code here? this could be in process or put all the code in the shooter part of the switch statement and switching between can be handed in game piece doesnt matter.
}

int Intake::returnBallCount(){
    return ballCount;
}

void Intake::countOfBalls(){
    if (stageOneFlag.Get() == true && ballCount != -1){
        ballCount++;
    }
}

void Intake::setBallCounterBroken(bool ballCounter){
    ballCounterBroken = ballCounter;
    if(ballCounter){
        ballCount = -1;
    }
}