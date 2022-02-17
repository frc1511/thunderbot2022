#include "GamEpiece.h"

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
        if(ballCount >= 0 ){ 
            currentBallCount = ballCount;
        }
        intakeDirection = NOTTAKE;
        shooterState = NOT_SHOOTING;
    }

}



void GamEpiece::process() {
    currentBallCount = intake.returnBallCount();
    switch(desiredShooterState){
        case(NOT_SHOOTING):
            if(shooterState == WARMUP_SHOOTER){
                shooterState = NOT_SHOOTING;
            }
            break;
        case(WARMUP_SHOOTER):
            if(shooterState == NOT_SHOOTING){
                shooterState = WARMUP_SHOOTER;
            }
            break;
        case(WANT_TO_SHOOT):
            if(shooterState != SHOOTING){
                shooterState = WANT_TO_SHOOT;
            }
            break;
        case(SHOOTING):

            break;
    }
    if(currentBallCount == 0){
        shooterState = NOT_SHOOTING;
    }
    if(shooterState == NOT_SHOOTING){
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
                if(shooterState == SHOOTING || shooterState == WANT_TO_SHOOT){
                    shooterState = WARMUP_SHOOTER;
                }
                break;
        }
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
                intake.setIntakeDirection(Intake::SHOOTING);
            }
            break;
        case(SHOOTING):
            if(intake.finishedShooting()){
                intake.setIntakeDirection(Intake::NOTTAKE);
                shooterState = WARMUP_SHOOTER;
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
    shooter.setShooterMode(shooterMode);
    desiredShooterState = WANT_TO_SHOOT;
}

void GamEpiece::setIntakeDirection(IntakeDirection intDir){ // intState is a local variable which is a "copy of intakeDirection"
    if(shooterState == NOT_SHOOTING){ // intake is part of the shooting process so make sure it isnt trying to move the ball when you intake
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
    Feedback::sendString("gamEpiece", "intakeDirection", intakeStateString.c_str());
    Feedback::sendString("gamEpiece", "shooterState", shooterStateString.c_str());

    Feedback::sendDouble("thunderdashboard","match_remaining", frc::DriverStation::GetMatchTime());
    Feedback::sendDouble("thunderdashboard", "inpitmode", isCraterMode);
}
