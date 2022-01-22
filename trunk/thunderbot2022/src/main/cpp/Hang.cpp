#include "Hang.h"
#include "string"

const double kHangWinchSpeed = .4;
const double kHangWinchSlowSpeed = kHangWinchSpeed*.5;
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed*.5;

Hang::Hang() {
reset();
}

Hang::~Hang() {}

void Hang::process() {
    
    switch(hangState)
    {
            case MID:
                if (highBarStep == 0 && wantToChange == true){
                    extend();
                }
                else if (highBarStep == 2){
                    retract();
                }
                else if(highBarStep == 3){
                    engageBrake();
                }
                break;
            case HIGH:
                if(highBarStep == 4 && wantToChange == true){
                    pivot();
                }
                else if(highBarStep == 6){
                    extend();
                }
                else if(highBarStep == 7){
                    reversePivot();
                }
                else if(highBarStep == 8){
                    retract();
                }
                else if(highBarStep == 9){
                    engageBrake();
                }
                break;
            case TRAVERSAL:
                if(highBarStep == 10 && wantToChange == true){
                    pivot();
                }
                else if(highBarStep == 12){
                    extend();
                }
                else if(highBarStep == 13){
                    reversePivot();
                }
                else if(highBarStep == 14){
                    retract();
                }
                else if(highBarStep == 15){
                    engageBrake();
                }
                break;
            case NOT_ON_BAR:
                engageBrake();
                winchMotor.Set(0);
                wantToChange = false;
                break;
    }
}

void Hang::reset() {
    int highBarStep = 0;
    bool isOnBar = false;
    hangState = NOT_ON_BAR;
    winchMotor.Set(0);
    brake.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot.Set(frc::DoubleSolenoid::Value::kReverse);
    isHangWorking = true;
    bool wantToChange = false;
}

void Hang::debug(Feedback *feedback) {
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

    feedback->sendBoolean("Hang", "is hang working", isHangWorking);
    feedback->sendDouble("Hang", "encoder value", winchEncoder.Get());
    feedback->sendDouble("Hang", "winch motor speed", winchMotor.Get());
    feedback->sendBoolean("Hang", "is on bar", isOnBar);
    feedback->sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    feedback->sendDouble("Hang", "mid bar step", midBarStep);
    feedback->sendDouble("Hang", "high/traversal bar step", highBarStep);
}

void Hang::pivot() {
    //toggles hang pivot piston to rotate extending arms forwards or backwards
    hangPivot.Toggle();
    highBarStep++;
}

void Hang::extend(){
/*
if the arm brake is engaged & the home sensor == 1, then the state is nothing
to change state to releasing brake, toggle the brake piston to extend brake
check if extended with flag sensors
*/
    if((homeSensor.Get() == 1) && (brake.Get() == frc::DoubleSolenoid::Value::kForward) && (leftFlag.Get() == 0) && (rightFlag.Get() == 0)){
        disengageBrake();
        isHangWorking = true;
        midBarStep++;
        highBarStep++;
    }
    else if(leftFlag.Get() == 0 && rightFlag.Get() == 0){
        isHangWorking == false;
        engageBrake();
    }
    else if(midBarStep == 1){
        midBarStep++;
        highBarStep++;
    }
}

void Hang::retract() {
/*if BOTH flag sensors == 1 && brake piston is disengaged
    rotate winch motor to max value w/encoder
    check that the homing sensor == 1
    engage brake
    */
   if((leftFlag.Get() == true) && (rightFlag.Get() == true) && (brake.Get() == frc::DoubleSolenoid::Value::kReverse) && (midBarStep == 2)){
        hangMaxHeight = winchEncoder.Get();
        hangSlowDownHeight = hangMaxHeight*.4;
        hangSlowDownMoreHeight = hangMaxHeight*.1;
       winchMotor.Set(kHangWinchSpeed);
       midBarStep++;
       isHangWorking = true;    
   }
   else if((leftFlag.Get() == false) || (rightFlag.Get() == false)){
       winchMotor.Set(0);
       isHangWorking = false;
   }
    else if(midBarStep == 3){
        if(winchEncoder.Get() == hangSlowDownHeight){
            midBarStep++;
            winchMotor.Set(kHangWinchSlowSpeed);
        }
    }
    else if(midBarStep == 4){
        if(winchEncoder.Get() == hangSlowDownMoreHeight){
            winchMotor.Set(kHangWinchSlowerSpeed);
            midBarStep++;
        }
    }
    else if(midBarStep == 5){
        if(winchEncoder.Get() == hangMaxHeight){
            winchMotor.Set(0);
            midBarStep++;
        }
    }        
    else if(midBarStep == 6){
        if(homeSensor.Get() == 1){
            winchMotor.Set(0);
            isOnBar = true;
            highBarStep++;
            midBarStep = 1000;
        }
    }
}

void Hang::reversePivot () {
//dont know how to do this yet, maybe slow exhaust. Check on Hang subteam later
    highBarStep++;
}

void Hang::engageBrake () {
    auto brakeEngageState = brake.Get();
    if(brakeEngageState == frc::DoubleSolenoid::Value::kReverse){
        highBarStep++;
    }
    else if(brakeEngageState == frc::DoubleSolenoid::Value::kForward){
        brake.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else {
    }
}

void Hang::disengageBrake() {
    auto brakeDisengageState = brake.Get();
    if(brakeDisengageState == frc::DoubleSolenoid::Value::kForward){
        highBarStep++;
    }
    else if(brakeDisengageState == frc::DoubleSolenoid::Value::kReverse){
        brake.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else {
    }
}

void Hang::changeState(){
/*
if the state is x and step is y and the robot is on bar
    change state to the next one
else{
    do nothing
}
*/
    if(hangState == MID && midBarStep >= 1000 && isOnBar == true){
        hangState = HIGH;
    }
    else if(hangState == HIGH && highBarStep == 9 && isOnBar == true){
        hangState = TRAVERSAL;
    }
    else{

    }
}
//create functions to get unused variables
/*
potential errors:
the mid steps happen multiple times, having a midBarStep and highBarStep changing the actions could cause issues
something seems wrong
*/