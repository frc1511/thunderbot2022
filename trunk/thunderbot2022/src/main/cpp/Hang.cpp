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

Hang::Hang() {}
Hang::~Hang() {}

void Hang::resetToMode(MatchMode mode)
{
    if (mode == MODE_TELEOP)
    {
        step = 0;
        targetStage = NOT_ON_BAR;
        winchMotor.Set(0);
        brake.Set(frc::DoubleSolenoid::Value::kForward);
        hangPivot1.Set(frc::DoubleSolenoid::Value::kReverse);
        hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
        hangTimer.Reset();
        retractStep = 0;
        extendStep = 0;
        brokenStep = 0;
        engageHardStop();
        disengageBrakeStep = 0;
    }
}

void Hang::sendFeedback()
{
    std::string stateString = "";
    switch (targetStage)
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

void Hang::process()
{

    switch (targetStage)
    {
    case STOP:
        step = 1000;
        retractStep = 100;
        extendStep = 100;
        break;
    case NOT_ON_BAR:
        engageBrake(); // adds 1 from the start
        winchMotor.Set(0);
        engageHardStop();
        break;
    case MID:
        step = 0;
        if (step == 0)
        {
            disengageHardStop();
        }
        else if (step == 1)
        {
            extendStep = 0;
            extend();
        }
        else if (step == 3)
        {
            retractStep = 0;
            retract();
        }
        else if (step == 4)
        {
            engageBrake();
        }
        else if (step == 5)
        {
            extendALittle();
        }
        else
        {
            winchMotor.Set(0);
        }
        break;
    case HIGH:
        if (step == 6)
        {
            extendALittle();
        }
        if (step == 7)
        {
            pivot();
            // hi jeff
        }
        else if (step == 8)
        {
            extendStep = 0;
            extend();
            // need a step to reset step
        }
        else if (step == 10)
        {
            reversePivot();
        }
        else if (step == 11)
        {
            retractStep = 0;
            retract();
        }
        else if (step == 12)
        {
            engageBrake();
        }
        else
        {
            winchMotor.Set(0);
        }
        break;
    case TRAVERSAL:
        if (step == 13)
        {
            extendALittle();
        }
        if (step == 14)
        {
            pivot();
        }
        else if (step == 15)
        {
            extendStep = 0;
            extend();
        }
        else if (step == 17)
        {
            reversePivot();
        }
        else if (step == 18)
        {
            retractStep = 0;
            retract(); // ISSUE: WHEN YOU RETRACT, THE PISTON NEEDS TO SHIFT
        }
        else if (step == 19)
        {
            engageBrake();
        }
        else
        {
            winchMotor.Set(0);
        }
        break;
    }
    switch (manual)
    {
    case EXTEND:
        brokenStep = 0;
        brokenExtend();
        break;
    case EXTEND_A_LITTLE:
        brokenExtendALittle();
        break;
    case RETRACT:
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

void Hang::engageHardStop()
{
    leftServo.Set(1);
    rightServo.Set(1);
}

void Hang::disengageHardStop()
{
    hangTimer.Reset();
    hangTimer.Start();
    leftServo.Set(1);
    rightServo.Set(1);
    if (hangTimer.Get().value() == kServoTime)
    {
        step++;
        hangTimer.Stop();
    }
}

void Hang::pivot()
{                  // adds 1 to step
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    hangTimer.Reset();
    hangTimer.Start();
    disengageHardStop();
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kForward);
    if (hangTimer.Get().value() == kPivotTime)
    {
        step++;
        hangTimer.Stop();
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
        extendStep++;
    }
    else if (leftFlag.Get() == 1 && rightFlag.Get() == 1)
    {
        engageBrake();
    }
    else if (extendStep == 1)
    {
        step++;
    }
}

void Hang::extendALittle() // ADDS 1 TO STEP
{
    if (homeSensor.Get() == 0)
    {
        double currentEncoderValue = winchEncoder.Get();
        disengageBrake();
        if (winchEncoder.Get() - currentEncoderValue == kExtendDifference)
        {
            engageBrake();
        }
        else if (winchEncoder.Get() - currentEncoderValue <= 50)
        {
        }
        else
        {
            engageBrake();
        }
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
        if (winchEncoder.Get() == 10)
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
{                         // adds 1 to step
    hangTimer.Reset();
    hangTimer.Start();
    hangPivot1.Set(frc::DoubleSolenoid::Value::kReverse);
    if(hangTimer.Get().value() == kReversePivotTime)
    {
        step++;
    }
}

void Hang::engageBrake() // needs timer
{                        // adds 1 to step
    /*auto brakeEngageState = brake.Get();
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
    }*/
    ratchetServo.Set(1);
    step++;
}

void Hang::disengageBrake() // needs timer
{
    /*auto brakeDisengageState = brake.Get();
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
    }*/
    if (rachetBeamBreak.Get() == 1 && disengageBrakeStep == 0)
    {
        winchMotor.Set(.05);
        ratchetServo.Set(0);
        disengageBrakeStep++;
    }
    else if (disengageBrakeStep == 1)
    {
        if (rachetBeamBreak.Get() == 1)
        {
            targetStage = STOP;
        }
        else if (rachetBeamBreak.Get() == 0)
        {
            winchMotor.Set(0);
            disengageBrakeStep = 0;
        }
    }
    else
    {
    }
}

// broken functions
void Hang::brokenExtend()
{
    if (winchEncoder.Get() <= 20 || winchEncoder.Get() >= 10)
    {
        disengageBrake();
        brokenStep++;
    }
    else if (winchEncoder.Get() >= 20 && brokenStep == 1)
    {
        engageBrake();
    }
}
void Hang::brokenRetract()
{
    double hangBrokenMaxHeight = winchEncoder.Get();
    double hangBrokenSlowHeight = hangBrokenMaxHeight * .3;
    double hangBrokenSlowerHeight = hangBrokenSlowHeight * .5;

    winchMotor.Set(kHangWinchSpeed);
    brokenStep++;

    if (brokenStep == 1)
    {
        if (winchEncoder.Get() == hangBrokenSlowHeight)
        {
            winchMotor.Set(kHangWinchSlowSpeed);
            brokenStep++;
        }
        else if (winchEncoder.Get() <= hangBrokenSlowHeight)
        {
        }
        else
        {
            winchMotor.Set(kHangWinchSlowSpeed);
        }
    }
    else if (brokenStep == 2)
    {
        if (winchEncoder.Get() == hangBrokenSlowerHeight)
        {
            winchMotor.Set(kHangWinchSlowerSpeed);
            brokenStep++;
        }
        else if (winchEncoder.Get() <= hangBrokenSlowerHeight)
        {
        }
        else
        {
            winchMotor.Set(kHangWinchSlowerSpeed);
        }
    }
    else if (brokenStep == 3)
    {
        if (winchEncoder.Get() == 10)
        {
            winchMotor.Set(0);
        }
        else if (winchEncoder.Get() >= 10)
        {
        }
        else
        {
            winchMotor.Set(0);
        }
    }
}

void Hang::brokenExtendALittle()
{
    if (winchEncoder.Get() <= 20 || winchEncoder.Get() >= 10)
    {
        double currentEncoderValue = winchEncoder.Get();
        disengageBrake();
        if (winchEncoder.Get() - currentEncoderValue == kExtendDifference)
        {
            engageBrake();
        }
        else if (winchEncoder.Get() - currentEncoderValue <= 50)
        {
        }
        else
        {
            engageBrake();
        }
    }
}

void Hang::targetBar(HangState stage)
{
    targetStage = stage;
}

void Hang::commandManual(Manual manualCommands)
{
    manual = manualCommands;
}