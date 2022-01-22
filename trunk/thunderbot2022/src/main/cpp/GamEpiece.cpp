#include "GamEpiece.h"

GamEpiece::GamEpiece(Limelight* limelight)
  : limelight(limelight) {
    //intake = new Intake();
    //storage = new Storage();
    //shooter = new Shooter();
    lastIntakeBeamValue = intakeBeam.Get();
}

GamEpiece::~GamEpiece() {

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

void GamEpiece::reset(int ballCount = -1) {
    if(ballCount >= 0 /*&& !intakeCounterBroken && !shooterCounterBroken*/){  
        currentBallCount = ballCount;
    }
    intakeState = NOTTAKE;
    shooterState = NOTSHOOTING;
    intake.reset();
    storage.reset();
    shooter.reset();
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


void GamEpiece::debug(Feedback* feedback){
    /*feedback->sendDouble("thunderdashboard", "ballcount", currentBallCount);

    switch(intakeState){
        case INTAKE:
            feedback->sendString("gamepiece", "intakeState", "INTAKE");
            break;
        case OUTTAKE:
            feedback->sendString("gamepiece", "intakeState", "OUTTAKE");
            break;
        case NOTTAKE:
            feedback->sendString("gamepiece", "intakeState", "NOTTAKE");
            break;
    }

    switch(shooterState){
        case NOTSHOOTING:
            feedback->sendString("gamepiece", "shooterState" , "NOTSHOOTING");
            break;
        case WANTTOSHOOT:
            feedback->sendString("gamepiece" , "shooterState" , "WANTTOSHOOT");
            break;
        case SHOOTING:
            feedback->sendString("gamepiece" , "shooterState" , "SHOOTING");
            break;
    }*/
}