#include "Hang.h"
#include "string"

const double kHangWinchSpeed = .4;
const double kHangWinchSlowSpeed = kHangWinchSpeed * .5;
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed * .5;

Hang::Hang() {}
Hang::~Hang() {}

void Hang::resetToMode(MatchMode mode)
{
    if (mode == MODE_TELEOP)
    {
        step = 0;
        hangState = NOT_ON_BAR;
        winchMotor.Set(0);
        brake.Set(frc::DoubleSolenoid::Value::kForward);
        hangPivot.Set(frc::DoubleSolenoid::Value::kReverse);
        pivotTimer.Reset();
        retractStep = 0;
        extendStep = 0;
    }
}

void Hang::sendFeedback()
{
    std::string stateString = "";
    switch (hangState)
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
    Feedback::sendDouble("Hang", "encoder value", winchEncoder.Get());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor.Get());
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
}

void Hang::process()
{

    switch (hangState)
    {
    case NOT_ON_BAR:
        engageBrake();//adds 1 from the start
        winchMotor.Set(0);
    break;
    case MID:
        step = 0;
        if (step == 0)
        {
            extend();
            extendStep = 0;
        }
        else if (step == 2)
        {
            retract();
            retractStep = 0;
        }
        else if (step == 3)
        {
            engageBrake();
        }
        break;
    case HIGH:
        if (step == 4)
        {
            pivot();
        //hi jeff
        }
        else if (step == 6)
        {
            extend();
            extendStep = 0;
            //need a step to reset step
        }
        else if (step == 7)
        {
            reversePivot();
        }
        else if (step == 8)
        {
            retract();
            retractStep = 0;
        }
        else if (step == 9)
        {
            engageBrake();
        }
        break;
    case TRAVERSAL:
        if (step == 10)
        {
            pivot();
        }
        else if (step == 11)
        {
            extend();
            extendStep = 0;
        }
        else if (step == 13)
        {
            reversePivot();
        }
        else if (step == 14)
        {
            retract();
            retractStep = 0;
        }
        else if (step == 15)
        {
            engageBrake();
        }
        break;
    }
}

void Hang::pivot()
{ // adds 1 to step
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    hangPivot.Toggle();
    pivotTimer.Start();
    if (pivotTimer.Get().value() == .5)
    {
        step++;
    }
}

void Hang::extend()
{ // adds 2 to step
    /*
    if the arm brake is engaged & the home sensor == 1, then the state is nothing
    to change state to releasing brake, toggle the brake piston to extend brake
    check if extended with flag sensors
    */
    if ((homeSensor.Get() == 1) && (brake.Get() == frc::DoubleSolenoid::Value::kForward) && (leftFlag.Get() == 0 && rightFlag.Get() == 0))
    {
        disengageBrake();
        extendStep++;
    }
    else if (leftFlag.Get() == 0 && rightFlag.Get() == 0)
    {
        engageBrake();
    }
    else if (extendStep == 1)
    {
        step++;
    }
}

void Hang::retract()
{ // adds 1 to step
    /*if BOTH flag sensors == 1 && brake piston is disengaged
        rotate winch motor to max value w/encoder
        check that the homing sensor == 1
        engage brake
        */
    if ((leftFlag.Get() == true) && (rightFlag.Get() == true) && (brake.Get() == frc::DoubleSolenoid::Value::kReverse))
    {
        hangMaxHeight = winchEncoder.Get();
        hangSlowDownHeight = hangMaxHeight * .4;
        hangSlowDownMoreHeight = hangMaxHeight * .1;
        winchMotor.Set(kHangWinchSpeed);
        retractStep++;
    }
    else if ((leftFlag.Get() == false) || (rightFlag.Get() == false))
    {
        winchMotor.Set(0);
    }
    else if (retractStep == 1)
    {
        if (winchEncoder.Get() == hangSlowDownHeight)
        {
            retractStep++;
            winchMotor.Set(kHangWinchSlowSpeed);
        }
    }
    else if (retractStep == 2)
    {
        if (winchEncoder.Get() == hangSlowDownMoreHeight)
        {
            winchMotor.Set(kHangWinchSlowerSpeed);
            retractStep++;
        }
    }
    else if (retractStep == 3)
    {
        if (winchEncoder.Get() == hangMaxHeight)
        {
            winchMotor.Set(0);
            retractStep++;
        }
    }
    else if (retractStep == 4)
    {
        if (homeSensor.Get() == 1)
        {
            winchMotor.Set(0);
            step++;
        }
    }
}

void Hang::reversePivot()
{ // adds 1 to step
    // dont know how to do this yet, maybe slow exhaust. Check on Hang subteam later
    step++;
}

void Hang::engageBrake()
{ // adds 1 to step
    auto brakeEngageState = brake.Get();
    if (brakeEngageState == frc::DoubleSolenoid::Value::kReverse)
    {
        step++;
    }
    else if (brakeEngageState == frc::DoubleSolenoid::Value::kForward)
    {
        brake.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
    }
}

void Hang::disengageBrake()
{
    auto brakeDisengageState = brake.Get();
    //hi jeff
    if (brakeDisengageState == frc::DoubleSolenoid::Value::kForward)
    {
    }
    else if (brakeDisengageState == frc::DoubleSolenoid::Value::kReverse)
    {
        brake.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
    }
}

void Hang::changeState(HangState stage){
    hangState = stage;
}