#include "Intake.h"

const double kSpeedStageOne = .7;
const double kSpeedStageTwo = .7;
const double kSpeedStageTwoSlow = .3;
Intake::Intake() : intakeMotorStageOne(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage1)),
                   intakeMotorStageTwo(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage2))
{
}

Intake::~Intake(){

}

void Intake::resetToMode(MatchMode mode){
    targetDirection = NOTTAKE;
    ballCount = 0;
}



void Intake::process(){
    // adds to ball count
    if(stageOneFlag.Get() == false){ // there is not ball in front
            stageOneLast = false;
        }
    if (stageOneFlag.Get() == true && stageOneLast == false){ // there is not a ball in front but there wasnt one
        // Do not change cell count away from UNKNOWN (-1)
        if (ballCount != -1){
            ballCount++;
        }
        stageOneLast = true;
    }
    // takes away from ball count
    if(shooterBallCount.Get() == true){ // there is a ball in front
            shooterBeamLast = true;
        }
    

    // if the ball count is a number it shouldnt be then make it the unknown value
    if(ballCount < 0 || ballCount > 2){
        ballCount = -1;
    }
    // the desired actions of the intake motors
    switch (targetDirection){
    case INTAKE:    //takes in balls and adjusts motor activation through the sensors
        leftIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        rightIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        intakePosition = true; // used for feedback
        if (stageTwoFlag.Get() == false){
            intakeMotorStageOne->Set(kSpeedStageOne);
            intakeMotorStageTwo->Set(kSpeedStageTwo);
        }
        else if (stageTwoFlag.Get() == true && stageOneFlag.Get() == false){
            intakeMotorStageOne->Set(kSpeedStageOne);
            intakeMotorStageTwo->Set(0);
        }
        else if (stageTwoFlag.Get() == true && stageOneFlag.Get() == true){
            intakeMotorStageTwo->Set(0);
            intakeMotorStageOne->Set(0);
        }

        break;
    case OUTTAKE:   // reverses stage 1 and stage 2 motors
        leftIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        rightIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        intakePosition = true; // used for feedback
        intakeMotorStageOne->Set(-kSpeedStageOne); // IDK what to put for reverse...so negatives
        intakeMotorStageTwo->Set(-kSpeedStageTwo);
        break;
    case NOTTAKE:   // motors are off
        leftIntake.Set(frc::DoubleSolenoid::Value::kForward); // IDK if I should use kReverse? I assume it is this though. yes
        rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        intakePosition = false; // used for feedback
        intakeMotorStageOne->Set(0);
        intakeMotorStageTwo->Set(0);
        break;
    case MANUAL:    // switches to manual control of the motors
        if(intakePosition){
            leftIntake.Set(frc::DoubleSolenoid::Value::kReverse);
            rightIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        }
        else{
            leftIntake.Set(frc::DoubleSolenoid::Value::kForward);
            rightIntake.Set(frc::DoubleSolenoid::Value::kForward);
        }
        intakeMotorStageOne->Set(intakeSpeed);
        intakeMotorStageTwo->Set(intakeSpeed);
        break;
    case SHOOTING:
            // exists so that it doesnt control stage 2 to do bad things

        if(shooterBallCount.Get() == false && shooterBeamLast == true){ // there is not a ball in front but there was; means a ball left the robot
            if(stageOneFlag.Get()){ // checks for a ball in stage one and starts to move it up if there is one
                intakeMotorStageOne->Set(intakeSpeed);
                moveBallUp = true;
            }
            else{ // if there isnt a ball in stage one you are done
                targetDirection = NOTTAKE;
            }

        }
        if(stageTwoFlag.Get() && moveBallUp){ // stops a ball if the stage one ball gets moved up
            moveBallUp = false;
            targetDirection = NOTTAKE;
        }

        break;
    }
    if (shooterBallCount.Get() == false && shooterBeamLast == true){ // there is not a ball in front but there was 
        // Do not change cell count away from UNKNOWN (-1)
        if (ballCount != -1){
            ballCount+= -1;
        }
        shooterBeamLast = false;
    }
}
// connects to gamEpiece
void Intake::setIntakeDirection(IntakeDirection intakeDirection){   
    //make sure you arent shooting when you try to tell intake what to do
    if(targetDirection != SHOOTING){
        targetDirection = intakeDirection;
    }
}

void Intake::setIntakePosition(bool position){
    intakePosition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed){
    intakeSpeed = speed;
} // positive to intake, negative to outtake

void Intake::giveBallToShooter(){
    // leave pistons where they were when you start to shoot and moves stage two to feed the ball
    intakeMotorStageTwo->Set(intakeSpeed);
    targetDirection = SHOOTING;
}
// returns the ball count
int Intake::returnBallCount(){
    return ballCount;
}
// adds a ball to the counter when the lower storage sensor is tripped
void Intake::countOfBalls(){
    if (stageOneFlag.Get() == true && ballCount != -1){
        ballCount++;
    }
}
// for if the sensor breaks
void Intake::setBallCounterBroken(bool ballCounter){
    ballCounterBroken = ballCounter;
    if(ballCounter){
        ballCount = -1;
    }
}

bool Intake::finishedShooting(){
    return targetDirection == NOTTAKE;
}

void Intake::sendFeedback(){ //debug
    std::string intakeState = "";
    switch(targetDirection){
        case(INTAKE):
            intakeState = "Intake";
            break;
        case(OUTTAKE):
            intakeState = "Outtake";
            break;
        case(NOTTAKE):
            intakeState = "Nottake";
            break;
        case(MANUAL):
            intakeState = "Manual";
            break;
        case(SHOOTING):
            intakeState = "Shooting";
            break;
    }
    Feedback::sendString("Intake", "Current target direction", intakeState.c_str());
    Feedback::sendBoolean("Intake", "State of Intake, true = up", intakePosition);
    Feedback::sendDouble("Intake", "Manual speed of intake/storage", intakeSpeed);
}
