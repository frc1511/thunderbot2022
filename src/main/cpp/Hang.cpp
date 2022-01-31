#include "Hang.h"
#include "string"

// hang retract speeds
const double kHangWinchSpeed = .4;
const double kHangWinchSlowSpeed = kHangWinchSpeed * .5;
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed * .5;
const double kExtendDifference = 30;
const double kRachetAndPawlBackdriveSpeed = .05;

// timer times
const double kPivotTime = .5;
const double kServoTime = .25;
const double kExtendTime = 2;
const double kReversePivotTime = .2;

//encoder constants
const double kEncoderMin = 10;
const double kBrokenExtendMax = 20;
const double kEncoderMax = 560;
const double kEncoderSlowHeight = kEncoderMax*.4;
const double kEncoderSlowerHeight = kEncoderSlowerHeight*.5;

Hang::Hang() {}
Hang::~Hang() {}

void Hang::resetToMode(MatchMode mode){
    if (mode == MODE_TELEOP){
        step = 0;
        targetStage = NOT_ON_BAR;
        manual = ENGAGE_BRAKE;
        winchMotor.Set(0);
        brake.Set(frc::DoubleSolenoid::Value::kForward);
        hangPivot1.Set(frc::DoubleSolenoid::Value::kReverse);
        hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
        hangTimer.Reset();
        retractStep = 0;
        extendStep = 0;
        brokenStep = 0;
        winchEncoder.Reset();
        engageBrake();
        bool autoDone = false;
    }
}

void Hang::sendFeedback(){
    std::string stateString = "";
    switch (targetStage){
        case MID:
            stateString = "going to/on mid";
            break;
        case HIGH_TRAVERSAL:
            stateString = "going to/on high";
            break;
        case NOT_ON_BAR:
            stateString = "not currently on a bar";
            break;
        case STOP:
            stateString = "stopped";
            break;
    }
    Feedback::sendString("Hang", "stage of robot motion", stateString.c_str());
    Feedback::sendDouble("Hang", "encoder value", winchEncoder.Get());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor.Get());
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
}

void Hang::process(){

    switch (targetStage){
        case STOP:
            step = 1000;
            retractStep = 100;
            extendStep = 100;
            winchMotor.Set(0);
            break;
        case NOT_ON_BAR:
            winchMotor.Set(0);
            break;
        case MID:
            if (step == 1)
            {
                extendStep = 0;
                step++;
            }
            else if(step == 2)
            {
                extend();
            }
            else if (step == 4)
            {
                engageBrake();
            }
            else if (step == 5)
            {
                retractStep = 0;
                step++;
            }
            else if(step == 6)
            {
                retract();
            }
            else
            {
                winchMotor.Set(0);
            }
            break;
        case HIGH_TRAVERSAL:
            if (step == 7 || step == 18)
            {
                extendALittle();
            }
            else if (step == 8 || step == 19)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
            }
            else if(step == 9 || step == 20)
            {
                pivot();
            }
            else if (step == 10 || step == 21)
            {
                extendStep = 0;
                step++;
            }
            else if (step == 11 || step == 22)
            {
                extend();
            }
            else if (step == 13 || step == 23)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
            }
            else if (step == 14 || step == 24)
            {
                reversePivot();
            }
            else if (step == 15 || step == 25)
            {
                engageBrake();
            }
            else if (step == 16 || step == 26)
            {
                retractStep = 0;
                step++;
            }
            else if(step == 17 || step == 27)
            {
                retract();
                autoDone = true;
            }
            else
            {
                winchMotor.Set(0);
            }
            break;
    }
    if(autoDone == false)
    {
        switch (manual)
        {
            case EXTEND:
                brokenStep = 0;
                brokenExtend();
                //SAME ISSUE WITH RESETTING VARIABLES
                break;
            case EXTEND_A_LITTLE:
                brokenExtendALittle();
                break;
            case RETRACT:
                hangBrokenMaxHeight = winchEncoder.Get();
                hangBrokenSlowHeight = hangBrokenMaxHeight * .3;
                hangBrokenSlowerHeight = hangBrokenSlowHeight * .5;
                brokenStep = 0;
                brokenRetract();
                break;
            case PIVOT:
                pivot();
                break;
            case REVERSE_PIVOT:
                reversePivot();
                break;
            case ENGAGE_BRAKE:
                engageBrake();
                break;
            case DISENGAGE_BRAKE:
                disengageBrake();
                break;
        }
    }
}

void Hang::pivot()
{ 
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kForward);
    if (hangTimer.Get().value() >= kPivotTime)
    {
        step++;
        hangTimer.Stop();
    }
}

void Hang::extend()
{ 
    /*
    if the arm brake is engaged & the home sensor == 1, then the state is nothing
    to change state to releasing brake, toggle the brake piston to extend brake
    check if extended with flag sensors
    */
    if ((homeSensor.Get() == 1) && (brake.Get() == frc::DoubleSolenoid::Value::kForward) )
    {
        extendStep++;
    }
    else if (extendStep == 1){
        step++;
    }
}

void Hang::extendALittle()
{
    if (homeSensor.Get() == 0){
        double currentEncoderValue = winchEncoder.Get();
        disengageBrake();
        if (winchEncoder.Get() - currentEncoderValue >= kExtendDifference)
        {
            engageBrake();
        }
        else if (winchEncoder.Get() - currentEncoderValue <= 50){
        }
        else{
            engageBrake();
        }
    }
}

void Hang::retract()
{ 
    /*if BOTH flag sensors == 1 && brake piston is disengaged
        rotate winch motor to max value w/encoder
        check that the homing sensor == 1
        engage brake
        */
    winchMotor.Set(kHangWinchSpeed);

    if (brake.Get() == frc::DoubleSolenoid::Value::kReverse)
    {
        retractStep++;
    }
    else if (retractStep == 1)
    {
        if (winchEncoder.Get() == kEncoderSlowHeight)
        {
            retractStep++;
            winchMotor.Set(kHangWinchSlowSpeed);
        }
    }
    else if (retractStep == 2)
    {
        if (winchEncoder.Get() == kEncoderSlowerHeight)
        {
            winchMotor.Set(kHangWinchSlowerSpeed);
            retractStep++;
        }
    }
    else if (homeSensor.Get() == 1)
    {
        winchMotor.Set(0);
        step++;
        winchEncoder.Reset();
    }
    else{
        winchMotor.Set(0);
    }
}

void Hang::reversePivot()
{ 
    hangPivot1.Set(frc::DoubleSolenoid::Value::kReverse);
    if (hangTimer.Get().value() >= kReversePivotTime)
    {
        step++;
    }
}

void Hang::engageBrake()
{                  
    ratchetServo.Set(1);
    step++;
}

void Hang::disengageBrake()
{
    winchMotor.Set(-.05);
    ratchetServo.Set(0);
}

void Hang::brokenExtend()
{
    if (winchEncoder.Get() <= kBrokenExtendMax || winchEncoder.Get() >= kEncoderMin)
    {
        disengageBrake();
        brokenStep++;
    }
    else if (winchEncoder.Get() >= 20 && brokenStep == 1){
        engageBrake();
    }
}

void Hang::brokenRetract(){

    winchMotor.Set(kHangWinchSpeed);
    brokenStep++;

    if (brokenStep == 1){
        if (winchEncoder.Get() == hangBrokenSlowHeight){
            winchMotor.Set(kHangWinchSlowSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 2){
        if (winchEncoder.Get() == hangBrokenSlowerHeight){
            winchMotor.Set(kHangWinchSlowerSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 3)
    {
        if (winchEncoder.Get() <= kEncoderMin)
        {
            winchMotor.Set(0);
        }
    }
}

void Hang::brokenExtendALittle()
{
    if (winchEncoder.Get() <= kBrokenExtendMax && winchEncoder.Get() >= kEncoderMin)
    {
        double currentEncoderValue = winchEncoder.Get();
        disengageBrake();
        if (winchEncoder.Get() - currentEncoderValue == kExtendDifference){
            engageBrake();
        }
        else if (winchEncoder.Get() - currentEncoderValue <= kBrokenExtendMax + 1)
        {
        }
        else{
            engageBrake();
        }
    }
}

void Hang::commandAuto()
{
    if(step == 1)
    {
        targetStage = MID;
    }
    else if(step == 4 || step == 11)
    {
        targetStage = HIGH_TRAVERSAL;
    }
}

void Hang::commandManual(Manual manualCommands){
    manual = manualCommands;
}
