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
        int step = 0;
        bool isOnBar = false;
        hangState = NOT_ON_BAR;
        winchMotor.Set(0);
        brake.Set(frc::DoubleSolenoid::Value::kForward);
        hangPivot.Set(frc::DoubleSolenoid::Value::kReverse);
        isHangWorking = true;
        bool wantToChange = false;
        pivotTimer.Reset();
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
    Feedback::sendBoolean("Hang", "is hang working", isHangWorking);
    Feedback::sendDouble("Hang", "encoder value", winchEncoder.Get());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor.Get());
    Feedback::sendBoolean("Hang", "is on bar", isOnBar);
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
    Feedback::sendBoolean("Hang", "is the button pressed to change", wantToChange);
}

void Hang::process()
{

    switch (hangState)
    {
    case MID:
        if (step == 0 && wantToChange == true)
        {
            extend();
        }
        else if (step == 2)
        {
            retract();
        }
        else if (step == 9)
        {
            engageBrake();
        }
        break;
    case HIGH:
        if (step == 10 && wantToChange == true)
        {
            pivot();
        //hi jeff
        }
        else if (step == 11)
        {
            extend();
        }
        else if (step == 13)
        {
            reversePivot();
        }
        else if (step == 14)
        {
            retract();
        }
        else if (step == 21)
        {
            engageBrake();
        }
        break;
    case TRAVERSAL:
        if (step == 22 && wantToChange == true)
        {
            pivot();
        }
        else if (step == 23)
        {
            extend();
        }
        else if (step == 25)
        {
            reversePivot();
        }
        else if (step == 26)
        {
            retract();
        }
        else if (step == 33)
        {
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
    if ((homeSensor.Get() == 1) && (brake.Get() == frc::DoubleSolenoid::Value::kForward) && (leftFlag.Get() == 0) && (rightFlag.Get() == 0))
    {
        disengageBrake();
        isHangWorking = true;
        step++;
    }
    else if (leftFlag.Get() == 0 && rightFlag.Get() == 0)
    {
        isHangWorking == false;
        engageBrake();
    }
    else if (step == 1)
    {
        step++;
    }
}

void Hang::retract()
{ // adds 7 to step
    /*if BOTH flag sensors == 1 && brake piston is disengaged
        rotate winch motor to max value w/encoder
        check that the homing sensor == 1
        engage brake
        */
    if ((leftFlag.Get() == true) && (rightFlag.Get() == true) && (brake.Get() == frc::DoubleSolenoid::Value::kReverse) && (step == 2))
    {
        hangMaxHeight = winchEncoder.Get();
        hangSlowDownHeight = hangMaxHeight * .4;
        hangSlowDownMoreHeight = hangMaxHeight * .1;
        winchMotor.Set(kHangWinchSpeed);
        step++;
        isHangWorking = true;
    }
    else if ((leftFlag.Get() == false) || (rightFlag.Get() == false))
    {
        winchMotor.Set(0);
        isHangWorking = false;
    }
    else if (step == 3)
    {
        if (winchEncoder.Get() == hangSlowDownHeight)
        {
            step++;
            winchMotor.Set(kHangWinchSlowSpeed);
        }
    }
    else if (step == 4)
    {
        if (winchEncoder.Get() == hangSlowDownMoreHeight)
        {
            winchMotor.Set(kHangWinchSlowerSpeed);
            step++;
        }
    }
    else if (step == 5)
    {
        if (winchEncoder.Get() == hangMaxHeight)
        {
            winchMotor.Set(0);
            step++;
        }
    }
    else if (step == 6)
    {
        if (homeSensor.Get() == 1)
        {
            winchMotor.Set(0);
            isOnBar = true;
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

void Hang::changeState()
{
    /*
    if the state is x and step is y and the robot is on bar
        change state to the next one
    else{
        do nothing
    }
    */
    if (hangState == MID && step == 10 && isOnBar == true && wantToChange == true)
    {
        hangState = HIGH;
    }
    else if (hangState == HIGH && step == 22 && isOnBar == true && wantToChange == true)
    {
        hangState = TRAVERSAL;
    }
    else
    {
    }
}