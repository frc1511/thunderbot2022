#include "GamEpiece.h"
#include <iostream>

#define SHOT_TIMER_TIME .5 //seconds

GamEpiece::GamEpiece(Limelight* limelight)
  : limelight(limelight) {
}

GamEpiece::~GamEpiece() {

}

void GamEpiece::doPersistentConfiguration() {
    intake.doPersistentConfiguration();
    shooter.doPersistentConfiguration();
}

void GamEpiece::resetToMode(MatchMode mode) {
    intake.resetToMode(mode);
    shooter.resetToMode(mode);

    if (mode == MODE_TELEOP || mode == MODE_AUTO) {
        double ballCount = Feedback::getEditableDouble("thunderdashboard", "starting_ball_count", -1);
        if(ballCount >= 0 ){ 
            currentBallCount = ballCount;
        }
        intakeDirection = NOTTAKE;
        shooterState = NOT_SHOOTING;
        desiredShooterState = NOT_SHOOTING;
    }

}



void GamEpiece::process() {
    currentBallCount = intake.returnBallCount();
    switch(desiredShooterState){
        case(NOT_SHOOTING): //Dont take it out of the process of shooting, so only takes it out if it is in warmup
            if(shooterState == WARMUP_SHOOTER){
                shooterState = NOT_SHOOTING;
            }
            break;
        case(WARMUP_SHOOTER): //dont take it out of the process of shooting, so only takes it out if it is not shooting
            if(shooterState == NOT_SHOOTING){
                shooterState = WARMUP_SHOOTER;
            }
            break;
        case(WANT_TO_SHOOT): // let them try to shoot as long as they weren't already shooting
            if(shooterState != SHOOTING){
                shooterState = WANT_TO_SHOOT;
            }
            break;
        case(SHOOTING): // it will never desire itself to be in shooting it just moves directly there, this just prevents a warning
            // hi peter
            break;
    }
    if(currentBallCount == 0 && (shooterState == WANT_TO_SHOOT || shooterState == NOT_SHOOTING)){
        shooterState = NOT_SHOOTING;
    }

    if(shooterState == NOT_SHOOTING || shooterState == WARMUP_SHOOTER){ // stage two is needed to intake and to shoot, so if you are trying to shoot dont let it intake
        switch(intakeDirection){
            case(INTAKE):
                intake.setIntakeDirection(Intake::INTAKE);
                break;
            case(OUTTAKE):
                intake.setIntakeDirection(Intake::OUTTAKE);
                break;
            case(NOTTAKE): 
                intake.setIntakeDirection(Intake::NOTTAKE);
                break;
            case(MANUAL):
                intake.setIntakeDirection(Intake::MANUAL);
                break;
        }
        shooter.spinInReverse(intakeDirection == OUTTAKE); // only reverse if outtaking
    }
    else{ // it is trying to shoot so dont reverse
        shooter.spinInReverse(false);
    }
    
    switch(shooterState){
        case(NOT_SHOOTING):
            shooter.setShooterSpinup(false);
            break;
        case(WARMUP_SHOOTER):
            shooter.setShooterSpinup(true);
            break;
        case(WANT_TO_SHOOT):
            desiredShooterState = WARMUP_SHOOTER;
            shooter.setShooterSpinup(true);
            if(shooter.isShooterReady() && intake.ballAtStageTwo()){ // waits until hood is in place and shooter is at speed
                shooterState = SHOOTING;
                intake.setIntakeDirection(Intake::SHOOTING);
                desiredShooterState = WARMUP_SHOOTER;
            }
            break;

            /**
            if(shotTimer.Get().value() >= SHOT_TIMER_TIME){
                    shooterState = SHOOTING;
                    intake.setIntakeDirection(Intake::SHOOTING);
                    desiredShooterState = WARMUP_SHOOTER;
                    shotTimer.Stop();
                    shotTimer.Reset();
                }
                shotTimer.Start();
            }
            else{
                shotTimer.Stop();
                shotTimer.Reset();*/
        case(SHOOTING):
            if(intake.finishedShooting()){ // intake shot a ball so it is done 
                intake.setIntakeDirection(Intake::NOTTAKE);
                shooterState = WARMUP_SHOOTER;
                desiredShooterState = WARMUP_SHOOTER;
            }
            break;
    }
    intake.process();
    shooter.process();
    
    
}

void GamEpiece::setShooterWarmUpEnabled(Shooter::ShooterMode shooterMode, bool enabled){
    if(enabled){ // if they want to warmup
        desiredShooterState = WARMUP_SHOOTER;
    }
    else{ // dont take it out of shooting/want to shoot becasue once started you cant stop    
        desiredShooterState = NOT_SHOOTING;
    }
    shooter.setShooterMode(shooterMode);
}

void GamEpiece::shootABall(Shooter::ShooterMode shooterMode){
    shooter.setShooterMode(shooterMode); // tells the shooter where to go
    desiredShooterState = WANT_TO_SHOOT; 
}

void GamEpiece::setIntakeDirection(IntakeDirection intDir){ // intState is a local variable which is a "copy of intakeDirection"
        intakeDirection = intDir;
}

void GamEpiece::setManualIntakeSpeed(double intakeSpeed){
    intakeDirection = MANUAL;
    intake.setIntakeSpeed(intakeSpeed);
}

void GamEpiece::setManualIntakePosition(bool intakePosition){
    intakeDirection = MANUAL;
    intake.setIntakePosition(intakePosition);
}

void GamEpiece::setManualHoodSpeed(double hoodSpeed){
    shooter.setHoodManual(hoodSpeed);
}

bool GamEpiece::ballAtStageOne(){
    return intake.ballAtStageOne();
}

bool GamEpiece::isShotInProgress(){
    return shooterState == SHOOTING || shooterState == WANT_TO_SHOOT;
}

int GamEpiece::getCurrentBallCount(){
    return currentBallCount;
}

void GamEpiece::changeShooterSpeed(bool increaseOrDecrease){
    shooter.changeManualSpeed(increaseOrDecrease);
}

void GamEpiece::cancelShot(){
    shooterState = NOT_SHOOTING;
    intakeDirection = NOTTAKE;
}

void GamEpiece::recordShooterValues(){
    shooter.recordShooterValues();
}

void GamEpiece::setShotOdometryMode(Shooter::OdometryMode mode) {
    shooter.setOdometryMode(mode);
}

void GamEpiece::sendFeedback() {
    intake.sendFeedback();
    shooter.sendFeedback();
    Feedback::sendDouble("thunderdashboard", "ballcount", currentBallCount);
    std::string intakeStateString = "";
    std::string shooterStateString = "";
    switch(intakeDirection){
        case INTAKE:
            intakeStateString = "INTAKE";
            break;
        case OUTTAKE:
            intakeStateString = "OUTTAKE";
            break;
        case NOTTAKE:
            intakeStateString = "NOTTAKE";
            break;
        case MANUAL:
            intakeStateString = "MANUAL";
    }

    switch(shooterState){
        case NOT_SHOOTING:
            shooterStateString = "NOT SHOOTING";
            break;
        case WARMUP_SHOOTER:
            shooterStateString = "WARMUP SHOOTER";
            break;
        case WANT_TO_SHOOT:
            shooterStateString = "WANT TO SHOOT";
            break;
        case SHOOTING:
            shooterStateString = "SHOOTING";
            break;
    }
    int currentAlliance = 0;
    switch(frc::DriverStation::GetAlliance()){
        case frc::DriverStation::kRed:
            currentAlliance = 0;
            break;
        case frc::DriverStation::kBlue:
            currentAlliance = 1;
            break;
        case frc::DriverStation::kInvalid:
            break;
    }
    Feedback::sendString("gamEpiece", "intakeDirection", intakeStateString.c_str());
    Feedback::sendString("gamEpiece", "shooterState", shooterStateString.c_str());
    Feedback::sendDouble("gamEpiece", "shot timer", shotTimer.Get().value());
    Feedback::sendDouble("gamEpiece", "game piece ball count", currentBallCount);

    Feedback::sendDouble("thunderdashboard","match_remaining", frc::DriverStation::GetMatchTime());
    Feedback::sendDouble("thunderdashboard","alliance",currentAlliance);
    Feedback::sendDouble("thunderdashboard", "inpitmode", isCraterMode);
}
