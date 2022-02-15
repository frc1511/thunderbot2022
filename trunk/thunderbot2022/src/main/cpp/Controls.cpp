#include "Controls.h"

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

#define SLOW_DRIVE_FACTOR .2
#define SLOW_ROTATE_FACTOR .3

#define AXIS_DEADZONE .05

Controls::Controls(Drive *drive, GamEpiece *gamEpiece, Hang *hang)
    : drive(drive), gamEpiece(gamEpiece), hang(hang){
}

Controls::~Controls()
{
}

void Controls::process()
{
    doSwitchPanel();
    doDrive();
#ifndef HOMER
    doAux();
#endif
}

void Controls::doDrive()
{
     // standard format for when you have two buttons to do opposite things
    /*if(extendHangY == true && retractHangA == false){ // Extends hang if isnt retracting
         hang->move(Hang::UP);
     }
     // Retracts hang if button pressed
     else if(retractHangA == true && extendHangY == false){ // Retracts hang if isnt extending
         hang->move(Hang::DOWN);
     }
     else{ // If both pressed or neither pressed it does nothing
         hang->move(Hang::STOP);
     }*/

    bool brickDrive = controllerDrive.GetRawButton(CROSS_BUTTON);
    bool alignWithHighHub = controllerDrive.GetRawButton(CIRCLE_BUTTON);
    bool toggleCamera = controllerDrive.GetRawButton(SQUARE_BUTTON);

    double xDriveVelocity = controllerDrive.GetRawAxis(LEFT_X_AXIS);
    double yDriveVelocity = controllerDrive.GetRawAxis(LEFT_Y_AXIS);
    double leftRotateVelocity = (controllerDrive.GetRawAxis(LEFT_TRIGGER) + 1) / 2;
    double rightRotateVelocity = (controllerDrive.GetRawAxis(RIGHT_TRIGGER) + 1) / 2;

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

    // TODO: Broken switches.

    bool fieldCentricBroken = false;

    bool driveDisabled = false;

    if (brickDrive)
    {
        //drive->makeBrick();
        driveDisabled = true;
    }

    if (alignWithHighHub)
    {
        //drive->cmdAlignToHighHub();
    }

    if (toggleCamera && !cameraWasToggled)
    {
        // TODO: Toggle the camera.
    }
    cameraWasToggled = toggleCamera;

    if (zeroRotation)
    {
        //drive->zeroRotation();
    }

    if (configOffsets && !offsetsWereConfigured)
    {
        //drive->configMagneticEncoders();
    }
    offsetsWereConfigured = configOffsets;

    if (fieldCentricBroken)
    {
        //drive->setControlMode(Drive::ROBOT_CENTRIC);
    }
    else
    {
        //drive->setControlMode(Drive::FIELD_CENTRIC);
    }

    double finalXVelocity = 0.0;
    double finalYVelocity = 0.0;
    double finalRotateVelocity = 0.0;

    // If rotate left.
    if (leftRotateVelocity != 0)
    {
        finalRotateVelocity -= leftRotateVelocity;
    }
    // If rotate left slow.
    else if (leftSlowRotateVelocity != 0)
    {
        finalRotateVelocity -= leftRotateVelocity * SLOW_ROTATE_FACTOR;
    }

    // If rotate right.
    if (rightRotateVelocity != 0)
    {
        finalRotateVelocity += rightRotateVelocity;
    }
    // If rotate right slow.
    else if (rightSlowRotateVelocity != 0)
    {
        finalRotateVelocity += rightRotateVelocity * SLOW_ROTATE_FACTOR;
    }

    // If drive in x direction.
    if (fabs(xDriveVelocity) > AXIS_DEADZONE)
    {
        finalXVelocity = xDriveVelocity;
    }
    // If drive slow in x direction.
    else if (fabs(xSlowDriveVelocity) > AXIS_DEADZONE)
    {
        finalXVelocity = xDriveVelocity * SLOW_DRIVE_FACTOR;
    }

    // If drive in y direction.
    if (fabs(yDriveVelocity) > AXIS_DEADZONE)
    {
        finalYVelocity = yDriveVelocity;
    }
    // If drive slow in y direction.
    else if (fabs(ySlowDriveVelocity) > AXIS_DEADZONE)
    {
        finalYVelocity = yDriveVelocity * SLOW_DRIVE_FACTOR;
    }

    if (!driveDisabled)
    {
        //drive->manualDrive(finalXVelocity, -finalYVelocity, -finalRotateVelocity);
    }
}

void Controls::doAux()
{
    // Normal Aux Controls
    if (controllerAux.GetRawButton(1))
    {
        lastPressedMode = Shooter::TARMAC_LINE;
    }
    else if (controllerAux.GetRawButton(3))
    {
        lastPressedMode = Shooter::LAUNCH_PAD;
    }
    else if (controllerAux.GetRawButton(2))
    {
        lastPressedMode = Shooter::ODOMETRY;
    }

    if (controllerAux.GetRawButton(8))
    {
        gamEpiece->setShooterWarmUpEnabled(lastPressedMode, true);
    }
    else if (controllerAux.GetRawButton(8) == false)
    {
        gamEpiece->setShooterWarmUpEnabled(lastPressedMode, false);
    }

    if (controllerAux.GetRawButton(6))
    {
        gamEpiece->shootABall(lastPressedMode);
    }

    if (controllerAux.GetRawButton(7))
    {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::INTAKE);
    }
    else if (controllerAux.GetRawButton(5))
    {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::OUTTAKE);
    }
    else
    {
        gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::NOTTAKE);
    }

    int dPadValue = controllerAux.GetPOV();

    if (hangActive == true) {
        // Turn off gamepiece when hanging
        gamEpiece->setIntakeDirection(GamEpiece::NOTTAKE);
        gamEpiece->setShooterWarmUpEnabled(Shooter::TARMAC_LINE, false);

        if (hangManual)
        {
            if (controllerAux.GetRawButtonPressed(3))
            {
                hang->commandManual(Hang::EXTEND);
            }
            else if (controllerAux.GetRawButtonPressed(1))
            {
                hang->commandManual(Hang::RETRACT);
            }
            else if (controllerAux.GetRawButtonPressed(4))
            {
                hang->commandManual(Hang::ENGAGE_BRAKE);
            }
            else if (controllerAux.GetRawButtonPressed(2))
            {
                hang->commandManual(Hang::EXTEND_A_LITTLE);
            }

            if (lastDPadValue == -1 && dPadValue == 0)
            {
                hang->commandManual(Hang::PIVOT);
            }
            else if (lastDPadValue == -1 && dPadValue == 180)
            {
                hang->commandManual(Hang::REVERSE_PIVOT);
            }
            else if (lastDPadValue == -1 && dPadValue == 90)
            {
                // Wind-Up Servo
            }
            else if (lastDPadValue == -1 && dPadValue == 270)
            {
                // Unwind Servo
            }
        } else if (controllerAux.GetRawButtonPressed(12))
        {
            hang->commandAuto();
        }
    } else {
        // Manual Aux Controls
        if (gamePieceManual == true)
        {
            gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::MANUAL);

            if (dPadValue == 180)
            {
                gamEpiece->setManualIntakePosition(true);
            }
            else if (dPadValue == 0)
            {
                gamEpiece->setManualIntakePosition(false);
            }

            if (dPadValue == 90)
            {
                gamEpiece->setManualIntakeSpeed(1);
            }
            else if (dPadValue == 270)
            {
                gamEpiece->setManualIntakeSpeed(-1);
            }
            if(dPadValue == -1){
                gamEpiece->setManualIntakeSpeed(0);
            }
        }
        else if (gamePieceManual == false)
        {
            if (controllerAux.GetRawButton(1))
            {
                lastPressedMode = Shooter::TARMAC_LINE;
            }
            else if (controllerAux.GetRawButton(3))
            {
                lastPressedMode = Shooter::LAUNCH_PAD;
            }
            else if (controllerAux.GetRawButton(2))
            {
                lastPressedMode = Shooter::ODOMETRY;
            }

            if (controllerAux.GetRawButton(8))
            {
                gamEpiece->setShooterWarmUpEnabled(lastPressedMode, true);
            }
            else if (controllerAux.GetRawButton(8) == false)
            {
                gamEpiece->setShooterWarmUpEnabled(lastPressedMode, false);
            }

            if (controllerAux.GetRawButton(6))
            {
                gamEpiece->shootABall(lastPressedMode);
            }

            if (controllerAux.GetRawButton(7))
            {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::INTAKE);
            }
            else if (controllerAux.GetRawButton(5))
            {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::OUTTAKE);
            }
            else
            {
                gamEpiece->setIntakeDirection(GamEpiece::IntakeDirection::NOTTAKE);
            }
        }
    }

    lastDPadValue = dPadValue;
}

void Controls::doSwitchPanel()
{
    hangActive = switchPanel.GetRawButton(6); // do be 8 tho
    gamePieceManual = switchPanel.GetRawButton(1);
    if (hangActive)
    {
        hangManual = switchPanel.GetRawButton(2);
    }
}
