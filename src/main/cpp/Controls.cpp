#include "Controls.h"

#define WATER_BUG_BUTTON      2 // GetRawButton() give bool
#define ALARM_CLOCK_BUTTON      3 // GetRawButton() give bool
#define T_STICK_BUTTON      1 // GetRawButton() give bool
#define SWING_SET_BUTTON      4 // GetRawButton() give bool
#define LEFT_BUMPER   5 // GetRawButton() give bool
#define RIGHT_BUMPER  6 // GetRawButton() give bool
#define BACK_BUTTON   9 // GetRawButton() give bool
#define START_BUTTON  10 // GetRawButton() give bool
#define LEFT_TRIGGER  3 // GetRawAxis() give double
#define RIGHT_TRIGGER 4 // GetRawAxis() give double
#define LEFT_TRIGGER_BUTTON 7  // GetRawButton() give bool
#define RIGHT_TRIGGER_BUTTON 8  // GetRawButton() give bool
#define LEFT_STICK_PRESS 11 // GetRawButton() give bool
#define RIGHT_STICK_PRESS 12 // GetRawButton() give bool
#define HOME_BUTTON 13 // GetRawButton() give bool
#define BIG_MIDDLE_BUTTON 14 // GetRawButton() give bool

#define LEFT_X_AXIS   0 // GetRawAxis() give double
#define LEFT_Y_AXIS   1 // GetRawAxis() give double
#define RIGHT_X_AXIS  2 // GetRawAxis() give double
#define RIGHT_Y_AXIS  5 // GetRawAxis() give double
#define DPAD          0 // GetPOV() give int

Controls::Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang) 
: drive(drive), gamEpiece(gamEpiece), hang(hang) {

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
    
}
