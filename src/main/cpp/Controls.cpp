#include "Controls.h"

// #define XBOX_CONTROLLER

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
#define SLOW_ROTATE_FACTOR .3

#define AXIS_DEADZONE .1

Controls::Controls(Drive *drive, GamEpiece *gamEpiece, Hang *hang)
    : drive(drive), gamEpiece(gamEpiece), hang(hang) {
    
}

Controls::~Controls() {
}

void Controls::resetToMode(MatchMode mode) {
    switch (mode) {
        case MODE_DISABLED:
        case MODE_AUTO:
            controllerDrive.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,  0);
            controllerDrive.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
            break;
        case MODE_TELEOP:
        case MODE_TEST:
            controllerDrive.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,  1);
            controllerDrive.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
            break;
    }
}

void Controls::process() {
    doSwitchPanel();
    doDrive();
#ifndef HOMER
    doAux();
#endif
}

void Controls::doDrive() {
    bool brickDrive = controllerDrive.GetRawButton(CROSS_BUTTON);
    bool alignWithHighHub = controllerDrive.GetRawButton(CIRCLE_BUTTON);
    bool toggleCamera = controllerDrive.GetRawButton(SQUARE_BUTTON);

    double xDriveVelocity = controllerDrive.GetRawAxis(LEFT_X_AXIS);
    double yDriveVelocity = controllerDrive.GetRawAxis(LEFT_Y_AXIS);
#ifdef XBOX_CONTROLLER
    double leftRotateVelocity = controllerDrive.GetRawAxis(LEFT_TRIGGER);
    double rightRotateVelocity = controllerDrive.GetRawAxis(RIGHT_TRIGGER);
#else
    double leftRotateVelocity = (controllerDrive.GetRawAxis(LEFT_TRIGGER) + 1) / 2;
    double rightRotateVelocity = (controllerDrive.GetRawAxis(RIGHT_TRIGGER) + 1) / 2;
#endif

    double xSlowDriveVelocity = controllerDrive.GetRawAxis(RIGHT_X_AXIS);
    double ySlowDriveVelocity = controllerDrive.GetRawAxis(RIGHT_Y_AXIS);
    double leftSlowRotateVelocity = controllerDrive.GetRawButton(LEFT_BUMPER);
    double rightSlowRotateVelocity = controllerDrive.GetRawButton(RIGHT_BUMPER);

    bool zeroRotation = controllerDrive.GetRawButton(OPTIONS_BUTTON);

    // Disable configue offsets because we don't want the drivers to mess it up.
#ifdef DRIVE_DEBUG
    bool configOffsets = controllerDrive.GetRawButton(SHARE_BUTTON);
#else
    bool configOffsets = false;
#endif

    bool driveDisabled = false;

    if (brickDrive) {
        drive->makeBrick();
        driveDisabled = true;
    }

    if (alignWithHighHub) {
        drive->cmdAlignToHighHub();
    }

    if (toggleCamera && !cameraWasToggled) {
        // TODO: Toggle the camera.
    }
    cameraWasToggled = toggleCamera;

    if (zeroRotation) {
        drive->zeroRotation();
    }

    if (configOffsets && !offsetsWereConfigured) {
        drive->configMagneticEncoders();
    }
    offsetsWereConfigured = configOffsets;

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
        finalXVelocity = xDriveVelocity;
        finalYVelocity = yDriveVelocity;
    }
    // If driving slow.
    else if (fabs(xSlowDriveVelocity) > AXIS_DEADZONE || fabs(ySlowDriveVelocity) > AXIS_DEADZONE) {
        finalXVelocity = xSlowDriveVelocity * SLOW_DRIVE_FACTOR;
        finalYVelocity = ySlowDriveVelocity * SLOW_DRIVE_FACTOR;
    }

    if (!driveDisabled) {
        drive->manualDrive(finalXVelocity, -finalYVelocity, -finalRotateVelocity);
    }
}

void Controls::doAux() {
    // Normal Aux Controls
    if (controllerAux.GetRawButton(1)) {
        lastPressedMode = Shooter::TARMAC_LINE;
    }
    else if (controllerAux.GetRawButton(3)) {
        lastPressedMode = Shooter::LAUNCH_PAD;
    }
    else if (controllerAux.GetRawButton(2)) {
        lastPressedMode = Shooter::ODOMETRY;
    }

    if (controllerAux.GetRawButton(8)) {
        gamEpiece->setShooterWarmUpEnabled(lastPressedMode, true);
    }
    else if (controllerAux.GetRawButton(8) == false) {
        gamEpiece->setShooterWarmUpEnabled(lastPressedMode, false);
    }

    if (controllerAux.GetRawButton(6)) {
        gamEpiece->shootABall(lastPressedMode);
    }

    if (controllerAux.GetRawButton(7)) {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::INTAKE);
    }
    else if (controllerAux.GetRawButton(5)) {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::OUTTAKE);
    }
    else {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::NOTTAKE);
    }

    int dPadValue = controllerAux.GetPOV();

    if (hangActive == true) {
        // Turn off gamepiece when hanging
        gamEpiece->setIntakeDirection(GamEpiece::NOTTAKE);
        gamEpiece->setShooterWarmUpEnabled(Shooter::TARMAC_LINE, false);
        if (!hangManual) {
            if (controllerAux.GetRawButtonPressed(RIGHT_STICK_PRESS)) {   
                hang->commandAuto();
            }
        } else {
            if (controllerAux.GetRawButtonPressed(3)) {
                hang->commandManual(Hang::EXTEND);
            }
            else if (controllerAux.GetRawButtonPressed(1)) {
                hang->commandManual(Hang::RETRACT);
            }
            else if (controllerAux.GetRawButtonPressed(4)) {
                hang->commandManual(Hang::ENGAGE_BRAKE);
            }
            else if (controllerAux.GetRawButtonPressed(2)) {
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
            else if(controllerAux.GetRawButton(SHARE_BUTTON)){
                hang->commandManual(Hang::REVERSE_PIVOT);
            }
        }
    }
    else {

        // Manual Aux Controls
        if (gamePieceManual == true) {
            gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::MANUAL);

            if (dPadValue == 180) {
                gamEpiece->setManualIntakePosition(true);
            }
            else if (dPadValue == 0 ) {
                gamEpiece->setManualIntakePosition(false);
            }

            double v = -1 * controllerAux.GetRawAxis(LEFT_Y_AXIS);
            if (v > 0.1 || v < -0.1) {
                gamEpiece->setManualIntakeSpeed(v);
            } else {
                gamEpiece->setManualIntakeSpeed(0);
            }

            if (dPadValue == 90) {
                gamEpiece->setManualHoodSpeed(1);
            }else if (dPadValue == 270) {            
                gamEpiece->setManualHoodSpeed(-1);
            }
            else{
                gamEpiece->setManualHoodSpeed(0);
            }


            if(controllerAux.GetRawButtonPressed(SHARE_BUTTON)){
                gamEpiece->changeShooterSpeed(false);
            }
            if(controllerAux.GetRawButtonPressed(OPTIONS_BUTTON)){
                gamEpiece->changeShooterSpeed(true);
            }


        }
        else if (gamePieceManual == false) {
            if (controllerAux.GetRawButton(SQUARE_BUTTON)) {
                lastPressedMode = Shooter::TARMAC_LINE;
            }
            else if (controllerAux.GetRawButton(CIRCLE_BUTTON)) {
                lastPressedMode = Shooter::LAUNCH_PAD;
            }
            else if (controllerAux.GetRawButton(CROSS_BUTTON)) {
                lastPressedMode = Shooter::ODOMETRY;
            }

            if (controllerAux.GetRawButton(RIGHT_TRIGGER_BUTTON)) {
                gamEpiece->setShooterWarmUpEnabled(lastPressedMode, true);
            }
            else {
                gamEpiece->setShooterWarmUpEnabled(lastPressedMode, false);
            }

            if (controllerAux.GetRawButton(RIGHT_BUMPER)) {
                gamEpiece->shootABall(lastPressedMode);
            }

            if (controllerAux.GetRawButton(LEFT_TRIGGER_BUTTON)) {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::INTAKE);
            }
            else if (controllerAux.GetRawButton(LEFT_BUMPER)) {
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
    hangActive = switchPanel.GetRawButton(6); // do be 8 tho
    gamePieceManual = switchPanel.GetRawButton(1);
    if (hangActive) {
        hangManual = switchPanel.GetRawButton(2);
    }
    isCraterMode = switchPanel.GetRawButton(10);
    robotCentric = switchPanel.GetRawButton(5);
    if (robotCentric) {
        drive->setControlMode(Drive::ROBOT_CENTRIC);
    }
    else {
        drive->setControlMode(Drive::FIELD_CENTRIC);
    }
}

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();
    return isCraterMode && controllerDrive.GetRawButton(TRIANGLE_BUTTON) &&
            controllerDrive.GetPOV() == 180 &&
            controllerAux.GetRawButton(CROSS_BUTTON) &&
            controllerAux.GetPOV() == 0;
}