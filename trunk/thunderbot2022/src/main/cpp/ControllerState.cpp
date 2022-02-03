#include "ControllerState.h"

// Xbox button maps
const int kAButton = 1; // GetRawButton() give bool
const int kBButton = 2; // GetRawButton() give bool
const int kXButton = 3; // GetRawButton() give bool
const int kYButton = 4; // GetRawButton() give bool
const int kLeftBumper = 5; // GetRawButton() give bool
const int kRightBumper = 6; // GetRawButton() give bool
const int kBackButton = 7; // GetRawButton() give bool
const int kStartButton = 8; // GetRawButton() give bool
const int kLeftTrigger = 2; // GetRawAxis() give float
const int kRightTrigger = 3; // GetRawAxis() give float
const int kLeftStickXAxis = 0; // GetRawAxis() give float
const int kLeftStickYAxis = 1; // GetRawAxis() give float
const int kRightStickXAxis = 4; // GetRawAxis() give float
const int kRightStickYAxis = 5; // GetRawAxis() give float
const int kDPad = 0; // GetPOV() give float

ControllerState::ControllerState(int controllerID):myController(controllerID){
    
}

ControllerState::~ControllerState(){

}

bool ControllerState::getRawButton(int buttonID){
    //put stuff here
    return false;
}

double ControllerState::getRawAxis(int axisID){
    //put stuff here
    return 0;
}

void ControllerState::setRawButton(int buttonID){
    //put stuff here
}

void ControllerState::setRawAxis(int axisID){
    //put stuff here
}
