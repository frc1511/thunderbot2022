#include "Hang.h"
#include "string"
#include "iostream"

// hang retract speeds
const double kHangWinchSpeed = .4;
const double kHangWinchSlowSpeed = kHangWinchSpeed * .5;
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed * .5;
const double kExtendDifference = 30;
const double kRachetAndPawlBackdriveSpeed = .05;

// timer times
const double kPivotTime = .5;
const double kReversePivotTime = .2;

//encoder constants
const double kEncoderMin = 10;
const double kBrokenExtendMax = 20;
const double kEncoderMax = 560;
const double kEncoderSlowHeight = kEncoderMax*.4;
const double kEncoderSlowerHeight = kEncoderSlowerHeight*.5;
const double kEncoderTolerance = 15;

Hang::Hang() : winchMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::Hang)) {}
Hang::~Hang() {}

void Hang::resetToMode(MatchMode mode){
    if (mode == MODE_TELEOP){
        step = 0;
        targetStage = NOT_ON_BAR;
        manual = ENGAGE_BRAKE;
        winchMotor->Set(0);
        hangPivot1.Set(frc::DoubleSolenoid::Value::kReverse);
        hangPivot2.Set(frc::DoubleSolenoid::Value::kForward);
        hangTimer.Reset();
        retractStep = 0;
        extendStep = 0;
        brokenStep = 0;
        winchMotor->SetEncoder(0);
        stringServo.Set(.5);
        ratchetServo.Set(0);
        bool autoDone = false;
        manual = NOT;
        controlsStep= 0;
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
    Feedback::sendDouble("Hang", "encoder value", winchMotor->GetEncoder());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor->Get());
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
}

void Hang::process(){
   
    switch (targetStage){
        case STOP:
            retractStep = 100;
            extendStep = 100;
            winchMotor->Set(0);
            break;
        case NOT_ON_BAR:
            winchMotor->Set(0);
            break;
        case MID:
            if (step == 0)
            {
                extendStep = 0;
                step++;
            }
            else if(step == 1)
            {   
                extend();
            }
            else if (step == 2)
            {
                engageBrake();
            }
            else if (step == 3)
            {
                retractStep = 0;
                step++;
            }
            else if(step == 4)
            {
                retract();
                controlsStep++;
            }
            else
            {
                winchMotor->Set(0);
                
            }
            break;
        case HIGH_TRAVERSAL:
            if (step == 0)
            {
                extendALittle();
            }
            else if (step == 1)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
            }
            else if(step == 2)
            {
                unwindString();
                pivot();
            }
            else if (step == 3)
            {
                extendStep = 0;
                step++;
            }
            else if (step == 4)
            {
                extend();
            }
            else if (step == 5)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
            }
            else if (step == 6)
            {
                reversePivot();
            }
            else if (step == 7)
            {
                engageBrake();
            }
            else if (step == 8)
            {
                retractStep = 0;
                step++;
            }
            else if(step == 9)
            {
                retract();
            }
            else if(step == 10)
            {
                pivot();
                step++;
                hangTimer.Reset();
            }
            else if(step == 11)
            {
                windUpString();
                autoDone = true;
                controlsStep++;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
    }
    if(autoDone == false)
    {
        switch (manual)
        {
            case EXTEND:
                brokenStep = 0;
                //brokenExtend();
                extend();
                break;
            case EXTEND_A_LITTLE:
               // brokenExtendALittle();
               extendALittle();
                break;
            /*case RETRACT:
                hangBrokenMaxHeight = winchMotor.GetEncoder();
                hangBrokenSlowHeight = hangBrokenMaxHeight * .3;
                hangBrokenSlowerHeight = hangBrokenSlowHeight * .5;
                brokenStep = 0;
                brokenRetract();
                break;*/
            case PULL_STRING:
                void windUpString();
                break;
            case PIVOT:
                if(isDone == false)
                {
                    pivot();
                }
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
            case RETRACT:
                retract();
              
                break;
            case NOT:
                break;
        }
    }
}

void Hang::pivot()
{ 
    isDone = true;
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    if(step%2 == 0){
    hangPivot1.Set(frc::DoubleSolenoid::kReverse);
    hangPivot2.Set(frc::DoubleSolenoid::kReverse);
    }
    else{
        hangPivot1.Set(frc::DoubleSolenoid::kForward);
        hangPivot2.Set(frc::DoubleSolenoid::kForward);
    }
    /*if (hangTimer.Get().value() >= kPivotTime)
    {
        step++;
        hangTimer.Stop();
    }*/
}

void Hang::extend()
{ 
    /*
    if the arm brake is engaged & the home sensor == 1, then the state is nothing
    to change state to releasing brake, toggle the brake piston to extend brake
    check if extended with flag sensors
    */
    if ((homeSensor.Get() == 1) && extendStep == 0)
    {
        disengageBrake();
        extendStep++;
    }
    else if (extendStep == 1){
        winchMotor->SetEncoder(kEncoderMax);
        extendStep++;
    }
    else if (extendStep == 2)
    {
        step++;
    }
}

void Hang::extendALittle()
{
    if (homeSensor.Get() == 0){
        //double currentEncoderValue = winchMotor.GetEncoder();  for real code
        disengageBrake();
        winchMotor->SetEncoder(50);
        if (winchMotor->GetEncoder() >= 50)
            /*winchMotor.GetEncoder() - currentEncoderValue >= kExtendDifference*/ 
        {
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
    if (retractStep == 0 && (winchMotor->GetEncoder() >= 550 && winchMotor->GetEncoder() <= 560))
    {
        winchMotor->Set(-kHangWinchSlowSpeed);
        retractStep++;
    }
    else if(retractStep == 1)
    {
        hangTimer.Reset();
        hangTimer.Start();
        retractStep++;
    }
    else if(hangTimer.Get().value() >= 5 && retractStep == 2)
    {
            retractStep++;
    }
    else if ((fabs(winchMotor->GetEncoder() - kEncoderSlowHeight) <= kEncoderTolerance) && (retractStep == 3))
    {
        winchMotor->Set(-kHangWinchSlowSpeed); 
        retractStep++;
        
    }
    else if (retractStep == 4)
    {
        if (fabs(winchMotor->GetEncoder()- kEncoderSlowerHeight) <= kEncoderTolerance)
        {
            winchMotor->Set(-kHangWinchSlowerSpeed);
        }
        else if(homeSensor.Get() == 1){
            winchMotor->Set(0);
            step++;
        }
    }
    else if(winchMotor->GetEncoder() <= 0)
    {
        winchMotor->Set(0);
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
    winchMotor->Set(-.05);
    ratchetServo.Set(0);
}

void Hang::brokenExtend()
{
    if (winchMotor->GetEncoder() <= kBrokenExtendMax && winchMotor->GetEncoder() >= kEncoderMin)
    {
        disengageBrake();
    }
    else if (winchMotor->GetEncoder() >= 20){
        engageBrake();
    }
}

void Hang::windUpString()
{
    stringServo.Set(0);
        if(hangTimer.Get().value() >= 2){
            stringServo.Set(.5);
            step++;
    }
}

void Hang::unwindString()
{
    stringServo.Set(1);
        if(hangTimer.Get().value() >= 2){
            stringServo.Set(.5);
            step++;
        }
}

void Hang::brokenRetract(){

    winchMotor->Set(kHangWinchSpeed);
    brokenStep++;

    if (brokenStep == 1){
        if (winchMotor->GetEncoder() == hangBrokenSlowHeight){
            winchMotor->Set(kHangWinchSlowSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 2){
        if (winchMotor->GetEncoder() == hangBrokenSlowerHeight){
            winchMotor->Set(kHangWinchSlowerSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 3)
    {
        if (winchMotor->GetEncoder() <= kEncoderMin)
        {
            winchMotor->Set(0);
        }
    }
}

void Hang::brokenExtendALittle()
{
    if (winchMotor->GetEncoder() <= kBrokenExtendMax && winchMotor->GetEncoder() >= kEncoderMin)
    {
        double currentEncoderValue = winchMotor->GetEncoder();
        disengageBrake();
        if (winchMotor->GetEncoder() - currentEncoderValue == kExtendDifference){
            engageBrake();
        }
        else if (winchMotor->GetEncoder() - currentEncoderValue <= kBrokenExtendMax + 1)
        {
        }
        else{
            engageBrake();
        }
    }
}

void Hang::commandAuto()
{
    step = 0;
}

void Hang::commandManual(Manual manualCommands){
    isDone = false;
    manual = manualCommands;
}