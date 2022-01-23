#include "GamEpiece.h"

GamEpiece::GamEpiece(Limelight* limelight)
  : limelight(limelight) {
    lastIntakeBeamValue = intakeBeam.Get();
}

GamEpiece::~GamEpiece() {

}

void GamEpiece::resetToMode(MatchMode mode) {
    intake.resetToMode(mode);
    storage.resetToMode(mode);
    shooter.resetToMode(mode);

    if (mode == MODE_TELEOP || mode == MODE_AUTO) {
        double ballCount = Feedback::getEditableDouble("thunderdashboard", "starting_ball_count", -1);
        if(ballCount >= 0 /*&& !intakeCounterBroken && !shooterCounterBroken*/){
            currentBallCount = ballCount;
        }
        intakeState = NOTTAKE;
        shooterState = NOTSHOOTING;
    }
}

void GamEpiece::sendFeedback() {
    /*feedback->sendDouble("thunderdashboard", "ballcount", currentBallCount);

    switch(intakeState){
        case INTAKE:
            feedback->sendString("gamEpiece", "intakeState", "INTAKE");
            break;
        case OUTTAKE:
            feedback->sendString("gamEpiece", "intakeState", "OUTTAKE");
            break;
        case NOTTAKE:
            feedback->sendString("gamEpiece", "intakeState", "NOTTAKE");
            break;
    }

    switch(shooterState){
        case NOTSHOOTING:
            feedback->sendString("gamEpiece", "shooterState" , "NOTSHOOTING");
            break;
        case WANTTOSHOOT:
            feedback->sendString("gamEpiece" , "shooterState" , "WANTTOSHOOT");
            break;
        case SHOOTING:
            feedback->sendString("gamEpiece" , "shooterState" , "SHOOTING");
            break;
    }*/
}

void GamEpiece::process() {
    if (!ballCounterBroken) { // ball counter works
        if(intakeBeam.Get() == true){ // there is a ball in front
            lastIntakeBeamValue = true;
        }
        if (intakeBeam.Get() == false && lastIntakeBeamValue == true){ // there is not a ball in front but it left
            // Do not change cell count away from UNKNOWN
            if (currentBallCount != BALL_COUNT_UNKNOWN){
                currentBallCount++;
            }
            lastIntakeBeamValue = false;
        }
    }
    switch(intakeState){
        case(INTAKE):
            // depends on how intake person does stuf
            break;
        case(OUTTAKE):
            // depends on how intake person does stuf
            break;
        case(NOTTAKE): 
            // depends on how intake person does stuf
            break;
        
    }
    switch(shooterState){
        case(NOTSHOOTING):
            // set something to make shooter not spin
            break;
        case(WANTTOSHOOT):
            if(currentBallCount != 0){
                // set something to make shooter spin
                // idk move to SHOOTING when shooter tells me it is ready and there is a ball

            }
            break;
        case(SHOOTING):
            // start shooting the ball
            // once the balls are done moving set it back to WANTTOSHOOT
            break;
    }


    intake.process();
    storage.process();
    shooter.process();
}

void GamEpiece::setIntakeState(IntakeState intState){ // intState is a local variable which is a "copy of intakeState"
    intakeState = intState;
}

void GamEpiece::setShooterState(ShooterState shootState){ // shootState is a local variable which is a "copy of ShooterState"
    if (shooterState != SHOOTING){ // makes sure it is not currently shooting when controls tries to change it to something else
        shooterState = shootState;
    }
}

void GamEpiece::setBallCounterBroken(bool ballCounter){
    ballCounterBroken = ballCounter;
    if(ballCounter){
        currentBallCount = BALL_COUNT_UNKNOWN;
    }
}