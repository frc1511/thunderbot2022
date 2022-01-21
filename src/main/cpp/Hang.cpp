#include "Hang.h"
#include "string"

const double kHangWinchSpeed = .3;
const double kHangWinchSlowSpeed = .2;
const double kHangWinchSlowerSpeed = .1;

const double kMaxHeight = 560;
const double kMinHeight = 0;
const double kSlowDownHeight = 500;
const double kSlowDownMoreHeight = 550;


Hang::Hang() {
reset();
}

Hang::~Hang() {}

void Hang::process() {
    
    switch(hangState)
    {
            case MID:
                break;
            case HIGH:
                break;
            case TRAVERSAL:
                break;
            case NOT_ON_BAR:
                break;
    }
}

void Hang::reset() {
    int step = 0;
    bool isOnBar = false;
    hangState = NOT_ON_BAR;
    winchMotor.Set(0);
    brake.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot.Set(frc::DoubleSolenoid::Value::kReverse);
    isHangWorking = FUNCTIONAL;
}

void Hang::debug() {
    std::string stateString = "";
    switch(hangState)
    {
        case MID:
            stateString = "going to/on mid";
            break;
        case HIGH:
            stateString = "going to/on high";
            break;
        case TRAVERSAL:
            stateString = "going to/on traversal";
            break;
        case NOT_ON_BAR:
            stateString = "not currently on a bar";
            break;
    }
}

void Hang::pivot() {
    //toggles hang pivot piston to rotate extending arms forwards or backwards
    hangPivot.Toggle();
}

void Hang::extend(){
/*
if the arm brake is engaged & the home sensor == 1, then the state is nothing
to change state to releasing brake, toggle the brake piston to extend brake
check if extended with flag sensors
*/
    if(homeSensor.Get() == 1 && brake.Get() == frc::DoubleSolenoid::Value::kForward){
        disengageBrake();
    }
}

void Hang::retract() {
/*if BOTH flag sensors == 1 && brake piston is disengaged
    rotate winch motor to max value w/encoder
    check that the homing sensor == 1
    engage brake
    */
   if((leftFlag.Get() == true) && (rightFlag.Get() == true) && (brake.Get() == frc::DoubleSolenoid::Value::kReverse) && (midBarStep == 0)){
       winchMotor.Set(kHangWinchSpeed);
       midBarStep++;
       isHangWorking = FUNCTIONAL;

       if(winchEncoder.Get() == kSlowDownHeight && midBarStep == 1){
           midBarStep++;
           winchMotor.Set(kHangWinchSlowSpeed);

           if(winchEncoder.Get() == kSlowDownMoreHeight && midBarStep == 2){
                winchMotor.Set(kHangWinchSlowerSpeed);
                midBarStep++;

                if(winchEncoder.Get() == kMaxHeight && midBarStep == 3){
                    winchMotor.Set(0);
                    midBarStep++;
                
                    if(homeSensor.Get() == 1 && midBarStep == 4){
                        winchMotor.Set(0);
                        isOnBar = true;
                        midBarStep + 1000;
                    }
                    else{

                    }
                }
                else{
                   
                }   
           }
           else{
            
            }
       }
       else{
          
        }
   }
   else{
       winchMotor.Set(0);
       isHangWorking = BROKEN;
   }
}

void Hang::reversePivot () {
//dont know how to do this yet, maybe slow exhaust. Check on Hang subteam later
}

void Hang::engageBrake () {
    auto brakeEngageState = brake.Get();
    if(brakeEngageState == frc::DoubleSolenoid::Value::kReverse){

    }
    else if(brakeEngageState == frc::DoubleSolenoid::Value::kForward){
        brake.Toggle();
    }
    else {

    }
}

void Hang::disengageBrake() {
    auto brakeDisengageState = brake.Get();
    if(brakeDisengageState == frc::DoubleSolenoid::Value::kForward){

    }
    else if(brakeDisengageState == frc::DoubleSolenoid::Value::kReverse){
        brake.Toggle();
    }
    else {

    }
}