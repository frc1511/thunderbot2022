#include "Controls.h"

#define WATER_BUG_BUTTON     2  // GetRawButton() give bool
#define ALARM_CLOCK_BUTTON   3  // GetRawButton() give bool
#define T_STICK_BUTTON       1  // GetRawButton() give bool
#define SWING_SET_BUTTON     4  // GetRawButton() give bool
#define LEFT_BUMPER          5  // GetRawButton() give bool
#define RIGHT_BUMPER         6  // GetRawButton() give bool
#define BACK_BUTTON          9  // GetRawButton() give bool
#define START_BUTTON         10 // GetRawButton() give bool
#define LEFT_TRIGGER         3  // GetRawAxis() give double
#define RIGHT_TRIGGER        4  // GetRawAxis() give double
#define LEFT_TRIGGER_BUTTON  7  // GetRawButton() give bool
#define RIGHT_TRIGGER_BUTTON 8  // GetRawButton() give bool
#define LEFT_STICK_PRESS     11 // GetRawButton() give bool
#define RIGHT_STICK_PRESS    12 // GetRawButton() give bool
#define HOME_BUTTON          13 // GetRawButton() give bool
#define BIG_MIDDLE_BUTTON    14 // GetRawButton() give bool

#define LEFT_X_AXIS   0 // GetRawAxis() give double
#define LEFT_Y_AXIS   1 // GetRawAxis() give double
#define RIGHT_X_AXIS  2 // GetRawAxis() give double
#define RIGHT_Y_AXIS  5 // GetRawAxis() give double
#define DPAD          0 // GetPOV() give int

#define AXIS_DEADZONE .05

#ifdef HOMER
Controls::Controls(Drive* drive) : drive(drive) {
#else
Controls::Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang) 
: drive(drive), gamEpiece(gamEpiece), hang(hang) {
#endif

}

Controls::~Controls() {

}

void Controls::process() {

    //standard format for when you have two buttons to do opposite things
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










     
    bool resetPosition           = controllerDriver.GetRawButton(SWING_SET_BUTTON);
    bool toggleDriveMode         = controllerDriver.GetRawButton(T_STICK_BUTTON);
    bool brickDrive              = controllerDriver.GetRawButton(WATER_BUG_BUTTON);
    bool configOffsets           = false; // Don't implement because 
    double xAxisVelocity         = controllerDriver.GetRawAxis(LEFT_X_AXIS);
    double yAxisVelocity         = controllerDriver.GetRawAxis(LEFT_Y_AXIS);
    double leftRotationVelocity  = (controllerDriver.GetRawAxis(LEFT_TRIGGER)+1)/2;
    double rightRotationVelocity = (controllerDriver.GetRawAxis(RIGHT_TRIGGER)+1)/2;
    int slowDriveDirection       = controllerDriver.GetPOV(DPAD);
    bool slowLeftVelocity        = controllerDriver.GetRawButton(LEFT_BUMPER);
    bool slowRightVelocity       = controllerDriver.GetRawButton(RIGHT_BUMPER);
    bool toggleSlowMode          = controllerDriver.GetRawButton(ALARM_CLOCK_BUTTON);

    // std::cout << xAxisVelocity << ", " << yAxisVelocity << '\n';

    bool disableDrive = false;

    if(resetPosition) {
        drive->zeroRotation();
    }


    if(toggleDriveMode) {
        if(!wasDriveModeToggled) {
            isFieldCentric = !isFieldCentric;
            drive->setControlMode(isFieldCentric ? Drive::FIELD_CENTRIC : Drive::ROBOT_CENTRIC);
        }
    }
    wasDriveModeToggled = toggleDriveMode;

    if(toggleSlowMode) {
        if(!wasSlowModeToggled) {
            slowModeEnabled = !slowModeEnabled;
        }
    }
    wasSlowModeToggled = toggleSlowMode;

    if (brickDrive) {
        drive->makeBrick();
        disableDrive = true;
    }

    if(configOffsets) {
        drive->configMagneticEncoders();
    }

    double finalXAxis = 0.0000000000;
    double finalYAxis = 0.0;
    double finalRotation = 0.0;

    // DPad not pressed, so normal mode.
    if(slowDriveDirection == -1) {
        if(slowModeEnabled){
            if(fabs(xAxisVelocity) > AXIS_DEADZONE) {
                finalXAxis = (.2 * xAxisVelocity);
            }
            if(fabs(yAxisVelocity) > AXIS_DEADZONE) {
                finalYAxis = (.2 * yAxisVelocity);
            }
        }
        else{
            if(fabs(xAxisVelocity) > AXIS_DEADZONE) {
                finalXAxis = xAxisVelocity;
                std::cout << finalXAxis << ", " << xAxisVelocity << "\n";
            }
            if(fabs(yAxisVelocity) > AXIS_DEADZONE) {
                finalYAxis = yAxisVelocity;
            }
        }
    }
    // DPad is pressed, so slow mode.
    else {
        // DPad Up.
        if((slowDriveDirection >= 375) || (slowDriveDirection <= 45)) {
            finalYAxis = -.1;
        }
        // DPad Down.
        if((slowDriveDirection >= 135) && (slowDriveDirection <= 225)) {
            finalYAxis = .1;
        }
        // DPad Right.
        if((slowDriveDirection >= 45) && (slowDriveDirection <= 135)) {
            finalXAxis = .1;
        }
        // DPad Left.
        if((slowDriveDirection >= 225) && (slowDriveDirection <= 315)) {
            finalXAxis = -.1;
        }
    }
    // Bumper pressed, so slow mode.
    if(slowLeftVelocity || slowRightVelocity) {
        if(slowLeftVelocity && !slowRightVelocity) {
            finalRotation = -.25;
        }
        if(slowRightVelocity && !slowLeftVelocity) {
            finalRotation = .25;
        }
    }
    // Bumper not pressed, so normal mode.
    else {
        if(slowModeEnabled){
            if((leftRotationVelocity > 0) && (rightRotationVelocity == 0)) {
                finalRotation = -(.3 * leftRotationVelocity);
            }
            if((rightRotationVelocity > 0) && (leftRotationVelocity == 0)) {
                finalRotation = (.3 * rightRotationVelocity);
            }
        }
        else{
            if((leftRotationVelocity > 0) && (rightRotationVelocity == 0)) {
                finalRotation = -leftRotationVelocity;
            }
            if((rightRotationVelocity > 0) && (leftRotationVelocity == 0)) {
                finalRotation = rightRotationVelocity;
            }
        }
    }

    if (!disableDrive) {
        drive->manualDrive(-finalXAxis, finalYAxis, finalRotation);
    }
    
}
