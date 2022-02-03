#include "GamEpiece.h"

const double kShooterStopped = 0;
const double kLaunchPadSpeed = 4;
const double kTarmacSpeed = 42;
const double kLaunchPadAngle = .2;
const double kTarmacAngle = .5;

GamEpiece::GamEpiece(Limelight* limelight)
  : limelight(limelight) {
}

GamEpiece::~GamEpiece() {

}

void GamEpiece::resetToMode(MatchMode mode) {
    intake.resetToMode(mode);
    shooter.resetToMode(mode);

    if (mode == MODE_TELEOP || mode == MODE_AUTO) {
        double ballCount = Feedback::getEditableDouble("thunderdashboard", "starting_ball_count", -1);
        if(ballCount >= 0 /*&& !intakeCounterBroken && !shooterCounterBroken*/){
            currentBallCount = ballCount;
        }
        intakeDirection = NOTTAKE;
        shooterState = NOT_SHOOTING;
    }

}



void GamEpiece::process() {
    currentBallCount = intake.returnBallCount();
    if(currentBallCount == 0){
        shooterState = NOT_SHOOTING;
    }
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
    
    switch(shooterState){
        case(NOT_SHOOTING):
            shooter.setShooterSpinup(false);
            break;
        case(WARMUP_SHOOTER):
            shooter.setShooterSpinup(true);
            break;
        case(WANT_TO_SHOOT):
            shooter.setShooterSpinup(true);
            
            if(shooter.isShooterReady()){
                shooterState = SHOOTING;
                intake.giveBallToShooter();

            }
            break;
        case(SHOOTING):
            if(intake.finishedShooting()){
                shooterState = WANT_TO_SHOOT;
            }
            break;
    }
    intake.process();
    shooter.process();
}

void GamEpiece::startWarmingUpShooter(Shooter::ShooterMode shooterMode){
    if(shooterState == NOT_SHOOTING){
        shooterState = WARMUP_SHOOTER;
    }
    shooter.setShooterMode(shooterMode);
}

void GamEpiece::startShootingTheBalls(Shooter::ShooterMode shooterMode){
    if(shooterState != SHOOTING){
        shooterState = WANT_TO_SHOOT;
    }
    shooter.setShooterMode(shooterMode);
}

void GamEpiece::stopShooting(){
    if(shooterState !=SHOOTING){
        shooterState = NOT_SHOOTING;
    }
}

void GamEpiece::setIntakeDirection(IntakeDirection intDir){ // intState is a local variable which is a "copy of intakeDirection"
    if(shooterState == NOT_SHOOTING){
        intakeDirection = intDir;
    }
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

/*void GamEpiece::setManualShooterSpeed(double shooterSpeed){
    desiredShooterSpeed += shooterSpeed;
}*/

void GamEpiece::setBallCounterBroken(bool ballCounter){
    intake.setBallCounterBroken(ballCounter);
}

void GamEpiece::sendFeedback() {
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
    Feedback::sendString("gamEpiece", "intakeDirection", intakeStateString.c_str());
    Feedback::sendString("gamEpiece", "shooterState", shooterStateString.c_str());
}
