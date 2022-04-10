#include "Controls.h"
#include <iostream>

// define moved to controllerstate

#ifdef XBOX_CONTROLLER

#define CROSS_BUTTON 1         // GetRawButton() give bool
#define CIRCLE_BUTTON 2        // GetRawButton() give bool
#define SQUARE_BUTTON 3        // GetRawButton() give bool
#define TRIANGLE_BUTTON 4      // GetRawButton() give bool
#define LEFT_BUMPER 5          // GetRawButton() give bool
#define RIGHT_BUMPER 6         // GetRawButton() give bool
#define SHARE_BUTTON 7         // GetRawButton() give bool
#define OPTIONS_BUTTON 8       // GetRawButton() give bool
#define LEFT_TRIGGER 2         // GetRawAxis() give double
#define RIGHT_TRIGGER 3        // GetRawAxis() give double
#define LEFT_TRIGGER_BUTTON 6  // GetRawButton() give bool
#define RIGHT_TRIGGER_BUTTON 7 // GetRawButton() give bool
#define LEFT_STICK_PRESS 9     // GetRawButton() give bool
#define RIGHT_STICK_PRESS 10   // GetRawButton() give bool
#define PLAYSTATION_BUTTON 11  // GetRawButton() give bool
#define TOUCHPAD_BUTTON 12     // GetRawButton() give bool

#define LEFT_X_AXIS 0  // GetRawAxis() give double
#define LEFT_Y_AXIS 1  // GetRawAxis() give double
#define RIGHT_X_AXIS 4 // GetRawAxis() give double
#define RIGHT_Y_AXIS 5 // GetRawAxis() give double
#define DPAD 0         // GetPOV() give int

#else

#define CROSS_BUTTON 2         // GetRawButton() give bool
#define CIRCLE_BUTTON 3        // GetRawButton() give bool
#define SQUARE_BUTTON 1        // GetRawButton() give bool
#define TRIANGLE_BUTTON 4      // GetRawButton() give bool
#define LEFT_BUMPER 5          // GetRawButton() give bool
#define RIGHT_BUMPER 6         // GetRawButton() give bool
#define SHARE_BUTTON 9         // GetRawButton() give bool
#define OPTIONS_BUTTON 10      // GetRawButton() give bool
#define LEFT_TRIGGER 3         // GetRawAxis() give double
#define RIGHT_TRIGGER 4        // GetRawAxis() give double
#define LEFT_TRIGGER_BUTTON 7  // GetRawButton() give bool
#define RIGHT_TRIGGER_BUTTON 8 // GetRawButton() give bool
#define LEFT_STICK_PRESS 11    // GetRawButton() give bool
#define RIGHT_STICK_PRESS 12   // GetRawButton() give bool
#define PLAYSTATION_BUTTON 13  // GetRawButton() give bool
#define TOUCHPAD_BUTTON 14     // GetRawButton() give bool

#define LEFT_X_AXIS 0  // GetRawAxis() give double
#define LEFT_Y_AXIS 1  // GetRawAxis() give double
#define RIGHT_X_AXIS 2 // GetRawAxis() give double
#define RIGHT_Y_AXIS 5 // GetRawAxis() give double
#define DPAD 0         // GetPOV() give int

#endif

#define SLOW_DRIVE_FACTOR .2
#define SLOW_ROTATE_FACTOR .1

#define AXIS_DEADZONE .1

Controls::Controls(Drive *drive, GamEpiece *gamEpiece, Hang *hang, Limelight* limelight, BlinkyBlinky* blinkyBlinky)
    : drive(drive), gamEpiece(gamEpiece), hang(hang), limelight(limelight), blinkyBlinky(blinkyBlinky) {
    
}

Controls::~Controls() {
}

void Controls::resetToMode(MatchMode mode) {
    auxController.chooseAutoMode(theAutoMode);
    driveController.chooseAutoMode(theAutoMode);
    auxController.reset();
    driveController.reset();
    switch (mode) {
        case MODE_DISABLED:
        case MODE_AUTO:
            
            break;
        case MODE_TELEOP:
        case MODE_TEST:
          
            break;
    }
}

void Controls::process() {
    doDrive();
    doSwitchPanel();
#ifndef HOMER
    doAux();
#endif
}

void Controls::doDrive() {
    driveController.process();
    bool brickDrive = driveController.getRawButton(CROSS_BUTTON);
    bool viceGrip = driveController.getRawButton(CIRCLE_BUTTON);
    bool toggleCamera = driveController.getRawButtonPressed(SQUARE_BUTTON);
    bool alignWithHighHub = driveController.getRawButton(TRIANGLE_BUTTON);

    double xDriveVelocity = driveController.getRawAxis(LEFT_X_AXIS);
    double yDriveVelocity = driveController.getRawAxis(LEFT_Y_AXIS);
#ifdef XBOX_CONTROLLER
    double leftRotateVelocity = driveController.getRawAxis(LEFT_TRIGGER);
    double rightRotateVelocity = driveController.getRawAxis(RIGHT_TRIGGER);
#else
    double leftRotateVelocity = (driveController.getRawAxis(LEFT_TRIGGER) + 1) / 2;
    double rightRotateVelocity = (driveController.getRawAxis(RIGHT_TRIGGER) + 1) / 2;
#endif

    double xSlowDriveVelocity = driveController.getRawAxis(RIGHT_X_AXIS);
    double ySlowDriveVelocity = driveController.getRawAxis(RIGHT_Y_AXIS);
    double leftSlowRotateVelocity = driveController.getRawButton(LEFT_BUMPER);
    double rightSlowRotateVelocity = driveController.getRawButton(RIGHT_BUMPER);

    bool zeroRotation = driveController.getRawButton(OPTIONS_BUTTON);
    bool calibrateGyro = driveController.getRawButton(SHARE_BUTTON);

    bool driveDisabled = false;

    if (brickDrive) {
        drive->makeBrick();
        driveDisabled = true;
    }

    if (viceGrip) {
        drive->setViceGrip(true);
    }
    else {
        drive->setViceGrip(false);
    }

    if (toggleCamera) {
        whichCamera = !whichCamera;
    }
    cameraWasToggled = toggleCamera;

    if (alignWithHighHub && !viceGrip) {
        drive->cmdAlignToHighHub();
    }

    if (zeroRotation) {
        drive->zeroRotation();
    }

    if (calibrateGyro) {
        drive->calibrateIMU();
        drive->zeroRotation();
    }

    double finalXVelocity = 0.0;
    double finalYVelocity = 0.0;
    double finalRotateVelocity = 0.0;

    // If rotate left.
    if (leftRotateVelocity != 0) {
        finalRotateVelocity -= leftRotateVelocity;
    }
    // If rotate left slow.
    else if (leftSlowRotateVelocity != 0) {
        finalRotateVelocity -= leftSlowRotateVelocity * SLOW_ROTATE_FACTOR;
    }

    // If rotate right.
    if (rightRotateVelocity != 0) {
        finalRotateVelocity += rightRotateVelocity;
    }
    // If rotate right slow.
    else if (rightSlowRotateVelocity != 0) {
        finalRotateVelocity += rightSlowRotateVelocity * SLOW_ROTATE_FACTOR;
    }

    // If driving regular.
    if (fabs(xDriveVelocity) > AXIS_DEADZONE || fabs(yDriveVelocity) > AXIS_DEADZONE) {
        if (fabs(xDriveVelocity) > AXIS_DEADZONE) {
            finalXVelocity = xDriveVelocity * .957;
        }
        if (fabs(yDriveVelocity) > AXIS_DEADZONE) {
            finalYVelocity = yDriveVelocity * .957;
        }
    }
    // If driving slow.
    else if (fabs(xSlowDriveVelocity) > AXIS_DEADZONE || fabs(ySlowDriveVelocity) > AXIS_DEADZONE) {
        if (fabs(xSlowDriveVelocity) > AXIS_DEADZONE) {
            finalXVelocity = xSlowDriveVelocity * SLOW_DRIVE_FACTOR;
        } 
        if (fabs(ySlowDriveVelocity) > AXIS_DEADZONE) {
            finalYVelocity = ySlowDriveVelocity * SLOW_DRIVE_FACTOR;
        }
    }

    std::pow(finalXVelocity, 3);
    std::pow(finalYVelocity, 3);

    if (!driveDisabled) {
        drive->manualDrive(finalXVelocity, -finalYVelocity, -finalRotateVelocity);
    }
}

void Controls::doAux() {
    // Normal Aux Controls
    auxController.process();
    int dPadValue = auxController.getRawAxis(6);

    if (hangActive == true) {
        // Turn off gamepiece when hanging
        gamEpiece->setIntakeDirection(GamEpiece::NOTTAKE);
        gamEpiece->setShooterWarmUpEnabled(Shooter::TARMAC_LINE, false);
        if (!hangManual) {
            if(auxController.getRawButtonPressed(LEFT_STICK_PRESS)) {
                hang->setCommandAutoOverride();
            }
            if (auxController.getRawButtonPressed(RIGHT_STICK_PRESS)) {   
                hang->commandAuto();
            }
            if(auxController.getRawButtonPressed(RIGHT_BUMPER)){
                hang->setPause(true, false);
            }
            if(auxController.getRawButtonPressed(LEFT_BUMPER)){
                hang->setPause(false, true);
            }
        } else {
            if (auxController.getRawButtonPressed(CIRCLE_BUTTON)) {
                hang->commandManual(Hang::DISENGAGE_BRAKE);
            }
            else if(auxController.getRawButtonReleased(CIRCLE_BUTTON)){
                hang->commandManual(Hang::ENGAGE_BRAKE);
            }
            else if (auxController.getRawButtonPressed(SQUARE_BUTTON)) {
                hang->commandManual(Hang::RETRACT);
            }
            else if (auxController.getRawButtonPressed(TRIANGLE_BUTTON)) {
                hang->commandManual(Hang::DISENGAGE_BRAKE);
            }
            else if (auxController.getRawButtonPressed(CROSS_BUTTON)) {
                hang->commandManual(Hang::EXTEND_A_LITTLE);
            }
            
            if (lastDPadValue == -1 && dPadValue == 0) {
                hang->commandManual(Hang::PIVOT_OUT);
            }
            else if (lastDPadValue == -1 && dPadValue == 180) {
                hang->commandManual(Hang::PIVOT_IN);
            }
            else if (lastDPadValue == -1 && dPadValue == 90) {
                hang->commandManual(Hang::PULL_STRING);
            }
            else if (lastDPadValue == -1 && dPadValue == 270) {
                hang->commandManual(Hang::UNWIND_STRING);
            }
            else if(auxController.getRawButton(SHARE_BUTTON)){
                hang->commandManual(Hang::REVERSE_PIVOT);
            }
            else if(auxController.getRawButton(OPTIONS_BUTTON)){
                hang->commandManual(Hang::DRIVE_DOWN);
            }
            else if(auxController.getRawButtonReleased(OPTIONS_BUTTON)){
                hang->commandManual(Hang::NOT);
            }
        }
    }
    else {
        
        // Manual and Normal Aux Controls
        if (auxController.getRawButton(SQUARE_BUTTON)) {
            lastPressedMode = Shooter::TARMAC;
            /*
            ShotMath::Shot shot = shotmath.calculateShot(units::meter_t(3.5), units::meters_per_second_t(0));
            */
        }
        else if (auxController.getRawButton(CIRCLE_BUTTON)) {
            if(nearOrFar){
                lastPressedMode = Shooter::NEAR_LAUNCH_PAD;
            }
            else{
                lastPressedMode = Shooter::FAR_LAUNCH_PAD;
            }
        }
        else if (auxController.getRawButton(CROSS_BUTTON)) {
            /*
            ShotMath::Shot shot = shotmath.calculateShot(units::meter_t(5), units::meters_per_second_t(1.2));
            */
           lastPressedMode = Shooter::ODOMETRY;
        }
        else if(auxController.getRawButton(TRIANGLE_BUTTON)) {
            if(highOrLow){
                lastPressedMode = Shooter::HIGH_HUB_SHOT;
            }
            else{
                lastPressedMode = Shooter::LOW_HUB_SHOT;
            }
        }
        /*
        if(auxController.getRawButtonPressed(SQUARE_BUTTON)){ //1
            auxController.record();
            driveController.record();
        }
        if(auxController.getRawButtonPressed(TRIANGLE_BUTTON)){ //4
            auxController.testStuff();
        }
        if(auxController.getRawButtonPressed(CIRCLE_BUTTON)){ //3
            auxController.replayAuto();
        }
        if(auxController.getRawButtonPressed(CROSS_BUTTON)){ //2
            auxController.clearAuto();
            driveController.clearAuto();
        }*/

        if (auxController.getRawButton(RIGHT_TRIGGER_BUTTON)) {
            gamEpiece->setShooterWarmUpEnabled(lastPressedMode, true);
            shoot = true;
        }
        else {
            gamEpiece->setShooterWarmUpEnabled(lastPressedMode, false);
            shoot = false;
        }
        if(auxController.getRawButtonPressed(TOUCHPAD_BUTTON)){
            gamEpiece->cancelShot();
        }
        if(auxController.getRawButtonPressed(LEFT_STICK_PRESS)){
            gamEpiece->setBallCount(0);
        }
        if(auxController.getRawButtonPressed(PLAYSTATION_BUTTON)){
            gamEpiece->setBallCount(1);
        }
        if(auxController.getRawButtonPressed(RIGHT_STICK_PRESS)){
            gamEpiece->setBallCount(2);
        }
        if (dPadValue == 90) {
            gamEpiece->setManualHoodSpeed(1);
            lastPressedMode = Shooter::MANUAL;
        }
        else if (dPadValue == 270) {            
            gamEpiece->setManualHoodSpeed(-1);
            lastPressedMode = Shooter::MANUAL;
        }
        else {
            gamEpiece->setManualHoodSpeed(0);
        }
        if(!isCraterMode){
            if(auxController.getRawButtonPressed(SHARE_BUTTON)){
                gamEpiece->changeShooterSpeed(false);
                lastPressedMode = Shooter::MANUAL;
            }
            if(auxController.getRawButtonPressed(OPTIONS_BUTTON)){
                gamEpiece->changeShooterSpeed(true);
                lastPressedMode = Shooter::MANUAL;
            }
        }
        else{
            if(auxController.getRawButtonPressed(SHARE_BUTTON)){
                gamEpiece->shooterPIDChange(false);
            }
            if(auxController.getRawButtonPressed(OPTIONS_BUTTON)){
                gamEpiece->shooterPIDChange(true);
            }
        }


        // Manual Aux Controls
        if (gamePieceManual == true) {
            if (dPadValue == 180) {
                gamEpiece->setManualIntakePosition(true);
            }
            else if (dPadValue == 0 ) {
                gamEpiece->setManualIntakePosition(false);
            }

            double v = -1 * auxController.getRawAxis(LEFT_Y_AXIS);
            if (v > 0.1 || v < -0.1) {
                gamEpiece->setManualIntakeSpeed(v);
            } else {
                gamEpiece->setManualIntakeSpeed(0);
            }

            
        }
        else if (gamePieceManual == false) {
            if (auxController.getRawButton(RIGHT_BUMPER)) {
                gamEpiece->shootABall(lastPressedMode);
            }
            if (auxController.getRawButton(LEFT_TRIGGER_BUTTON)) {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::INTAKE);
            }
            else if (auxController.getRawButton(LEFT_BUMPER)) { //auxController.getRawButton(LEFT_BUMPER)
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::OUTTAKE);
            }
            else {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::NOTTAKE);
            }
        }
    }

    lastDPadValue = dPadValue;
}

void Controls::doSwitchPanel() {
    hangActive = switchPanel.GetRawButton(8); 
    gamePieceManual = switchPanel.GetRawButton(1);
    if (hangActive) {
        hangManual = switchPanel.GetRawButton(3);
        hang->setIsLow(switchPanel.GetRawButton(4));
        hang->setGoingForHigh(switchPanel.GetRawButton(2)); // pressed means high bar
    }
    isCraterMode = switchPanel.GetRawButton(10);
    robotCentric = switchPanel.GetRawButton(5);
    if (robotCentric) {
        drive->setControlMode(Drive::ROBOT_CENTRIC);
    }
    else {
        drive->setControlMode(Drive::FIELD_CENTRIC);
    }
    highOrLow = switchPanel.GetRawButton(7);
    nearOrFar = switchPanel.GetRawButton(6);
    recordController = switchPanel.GetRawButtonPressed(12) /*|| switchPanel.GetRawButtonReleased(12)*/;
    clearController = switchPanel.GetRawButtonPressed(11) /*|| switchPanel.GetRawButtonReleased(11)*/;
    if(clearController){
        driveController.clearAuto();
        auxController.clearAuto();
    }
    else if(recordController){
        driveController.record();
        auxController.record();
    }
    peterCentric = switchPanel.GetRawButton(9);
    gamEpiece->setShotOdometryMode(Shooter::INTERPOLATION);

    if (getCurrentMode() == MODE_DISABLED || hangActive) {
        limelight->setLEDMode(Limelight::LEDMode::OFF);
    }
    else {
        limelight->setLEDMode(Limelight::LEDMode::ON);
    }

    if (!drive->getIMUCalibrated()) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::CALIBRATING);
    }
    else if (getCurrentMode() == MODE_DISABLED) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::DISABLED);
    }
    else if (isCraterMode) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::CRATER_MODE);
    }
    else if (hangActive) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::HANGER_STATUS);
    }
    else if (peterCentric) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::GAMePIECE);
    }
    else {
        blinkyBlinky->setLEDMode(BlinkyBlinky::ALLIANCE);
    }
    
}

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();
    return isCraterMode && driveController.getRawButton(TRIANGLE_BUTTON) &&
            driveController.getRawAxis(6) == 180 &&
            auxController.getRawButton(CROSS_BUTTON) &&
            auxController.getRawAxis(6) == 0;
}

void Controls::controllerInDisable(){
    driveController.process();
#ifndef HOMER
    auxController.process();
#endif
    if(driveController.getRawButton(SHARE_BUTTON)){
        drive->calibrateIMU();
        drive->zeroRotation();
        drive->resetOdometry();
    }
    doSwitchPanel();
}

void Controls::autoForTrevor(){
    auxController.replayAuto();
    driveController.replayAuto();
    std::cout << "shooting :D\n";
    /*lastPressedMode = Shooter::LOW_HUB_SHOT;
    gamEpiece->setShooterWarmUpEnabled(Shooter::LOW_HUB_SHOT, true);
    gamEpiece->shootABall(Shooter::LOW_HUB_SHOT);*/
}

void Controls::chooseAutoMode(int autoMode){
    driveController.chooseAutoMode(autoMode);
    auxController.chooseAutoMode(autoMode);
    theAutoMode = autoMode;
    std::cout << autoMode << "\n";
}
void Controls::sendFeedback(){
    driveController.sendFeedback();
    auxController.sendFeedback();
    shotmath.Feedback();
    blinkyBlinky->sendFeedback();
    std::string mode = "";
    switch(lastPressedMode){
        case(Shooter::TARMAC_LINE):
            mode = "tarmac line";
            break;
        case(Shooter::FAR_LAUNCH_PAD):
            mode = "far launch pad";
            break;
        case(Shooter::NEAR_LAUNCH_PAD):
            mode = "near launch pad";
            break;
        case(Shooter::HIGH_HUB_SHOT):
            mode = "high hub shot";
            break;
        case(Shooter::LOW_HUB_SHOT):
            mode = "low hub shot";
            break;
        case(Shooter::ODOMETRY):
            mode = "odometry";
            break;
        case(Shooter::MANUAL):
            mode = "manual";
            break;
        case(Shooter::TARMAC):
            mode = "tarmac";
            break;
    }
    Feedback::sendString("controls", "last pressed shooter mode", mode.c_str());
    Feedback::sendDouble("thunderdashboard", "frontcamera", whichCamera);
    Feedback::sendBoolean("controls", "warmup, true = yes", shoot);
    Feedback::sendDouble("controls", "whichAutoMode", theAutoMode);
    Feedback::sendDouble("thunderdashboard", "inpitmode", isCraterMode);
}