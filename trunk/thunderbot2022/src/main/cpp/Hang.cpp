#include "Hang.h"
#include "string"
#include "iostream"

// hang retract speeds
//kHangWinchSpeed is for the initial retracting, going from fully extended to partially retracted
const double kHangWinchSpeed = .75;
//hangWinchSlowSpeed is for the next step, slowing down the arms
const double kHangWinchSlowSpeed = kHangWinchSpeed * .5;
//hangWinchSlowerSpeed slows the arms even more
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed * .5;
//difference between the current encoder value and what it was before the arms extended a little to get off the bar
const double kExtendDifference = 30;
//self explanatory
const double kRachetAndPawlBackdriveSpeed = .05;

// timer times
const double kPivotTime = .5;
const double kReversePivotTime = .2;

//encoder constants
//minimum the encoder will go
const double kEncoderMin = .1;
//safety for extending - make sure that the arms are retracted enough before releasing the brake
const double kBrokenExtendMax = .15;
//max height
const double kEncoderMax = 6;
//height corresponding to kHangWinchSlowSpeed
const double kEncoderSlowHeight = kEncoderMax*.4;
//height corresponding to kHangWinchSlowerSpeed
const double kEncoderSlowerHeight = kEncoderSlowerHeight*.5;
//tolerance for maximum retract
const double kEncoderTolerance = .1;
//hi ishan
//servo speed constants
const double kServoBackward = 0;
const double kServoForward = 1;
const double kServoStopped = .5;

Hang::Hang() : winchMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::Hang)) {
    winchMotor->ConfigAlternateEncoder(2048);
}
Hang::~Hang() {}

void Hang::doPersistentConfiguration(){
    winchMotor->RestoreFactoryDefaults();
    // brake? coast?
    //winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
    winchMotor->BurnFlash();
}

void Hang::resetToMode(MatchMode mode){
    step = 0;
    targetStage = NOT_ON_BAR;
    manual = ENGAGE_BRAKE;
    winchMotor->Set(0);
    //pistons are the opposite of what you think
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
    hangTimer.Reset();
    retractStep = 0;
    extendStep = 0;
    brokenStep = 0;
    resetEncoder();
    stringServo.Set(kServoStopped);
    ratchetServo.Set(0);
    autoDone = false;
    manual = NOT;
    stepDone = true;
    currentEncoderValue = 0;



}

void Hang::sendFeedback(){
    std::string targetString = "";
    switch (targetStage){
        case MID:
            targetString = "going to/on mid";
            break;
        case HIGH_TRAVERSAL:
            targetString = "going to/on high";
            break;
        case NOT_ON_BAR:
            targetString = "not currently on a bar";
            break;
        case STOP:
            targetString = "stopped";
            break;
    }
    int hangStatus = 0;
    if(stepDone){
        hangStatus = 2;
    }
    else{
        hangStatus = 1;
    }
    Feedback::sendDouble("Hang", "encoder value", readEncoder());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor->Get());
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
    Feedback::sendBoolean("Hang", "Home sensor value", homeSensor.Get());
    Feedback::sendString("Hang", "stage robot is going to", targetString.c_str());
    Feedback::sendDouble("Hang", "extend step value", extendStep);
    Feedback::sendDouble("Hang", "retract step", retractStep);

    Feedback::sendDouble("thunderdashboard", "hang_bar", hangBar);
    Feedback::sendDouble("thunderdashboard", "hang_status", hangStatus);
}

void Hang::process(){
#ifdef TEST_BOARD
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
                hangBar = 1;
                extendStep = 0;
                step++;
                std::cout << "reset" << '\n';
            }
            else if(step == 1)
            {   
                extend();
                std::cout << "extending" << '\n';
            }
            else if (step == 2)
            {
                engageBrake();
                std::cout << "engage brake" << '\n';
            }
            else if (step == 3)
            {
                retractStep = 0;
                extendStep = 0;
                step++;
                std::cout << "reset" << '\n';
            }
            else if(step == 4)
            {
                retract();
                stepDone = true;
                //std::cout << "retract" << '\n';
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
                std::cout << "extendALittle" << '\n';
                stepDone = false;
                autoDone = false;
            }
            else if (step == 1)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                std::cout << "resetting" << '\n';
            }
            else if(step == 2)
            {
                unwindString();
                pivot(true);
                std::cout << "pivot" << '\n';
            }
            else if (step == 3)
            {
                extendStep = 0;
                step++;
                std::cout << "resetting" << '\n';
            }
            else if (step == 4)
            {
                extend();
                std::cout << "extending" << '\n';
            }
            else if (step == 5)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                std::cout << "resetting" << '\n';
            }
            else if (step == 6)
            {
                reversePivot();
                std::cout << "reverse pivot" << '\n';
            }
            else if (step == 7)
            {
                engageBrake();
                std::cout << "engage brake" << '\n';
            }
            else if (step == 8)
            {
                retractStep = 0;
                step++;
                std::cout << "reset" << '\n';
            }
            else if(step == 9)
            {
                retract();
                std::cout << "retract" << '\n';
            }
            else if(step == 10)
            {
                pivot(false);
                step++;
                hangTimer.Reset();
                std::cout << "pivot" << '\n';
            }
            else if(step == 11)
            {
                windUpString();
                autoDone = true;
                stepDone = true;
                std::cout << "windUpString" << '\n';
                extendALittleDone = false;
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
            case RETRACT:
                hangBrokenMaxHeight = readEncoder();
                hangBrokenSlowHeight = hangBrokenMaxHeight * .3;
                hangBrokenSlowerHeight = hangBrokenSlowHeight * .5;
                brokenStep = 0;
                brokenRetract();
                break;
            case PULL_STRING:
                windUpString();
                break;
            case UNWIND_STRING:
                unwindString();
                break;
            case PIVOT_OUT:
                if(isDone == false)
                {
                    pivot(true);
                }
                break;
            case PIVOT_IN:
                if(isDone == false)
                {
                    pivot(false);
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
            case NOT:
                break;
        }
    }
#endif
}

void Hang::pivot(bool armsForward)
{ 
    isDone = true;
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    if(armsForward == false){
    hangPivot1.Set(frc::DoubleSolenoid::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::kForward);
    }
    else{
        hangPivot1.Set(frc::DoubleSolenoid::kReverse);
        hangPivot2.Set(frc::DoubleSolenoid::kReverse);
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
    if ((homeSensor.Get() == 1 || extendALittleDone == true)  && extendStep == 0 )
    {
        disengageBrake();
        extendStep++;
    }
    else if (extendStep == 1){
        winchMotor->Set(.5);
        extendStep++;
    }
    else if (extendStep == 2 && (readEncoder() >= 5.5 && readEncoder() <= 6))
    {
        step++;
    }
}

void Hang::extendALittle()
{
    if (homeSensor.Get() == 1 && extendStep == 0){
    //currentEncoderValue = winchMotor->GetEncoder();
        std::cout << "got the encoder" << '\n';
        disengageBrake();
        std::cout << "disenaging brake" << '\n';
        extendStep++;
        winchMotor->Set(.6);
        std::cout << currentEncoderValue << '\n';
        extendStep++;
    }
    else if (readEncoder() >= kEncoderTolerance && extendStep == 1)
    {
        winchMotor->Set(0);
        engageBrake();
        std::cout << "completed" << '\n';
        extendStep++;
    }
    else if(extendStep == 2){
        step++;
        extendALittleDone = true;
    }
}

void Hang::retract()
{ 
    /*if BOTH flag sensors == 1 && brake piston is disengaged
        rotate winch motor to max value w/encoder
        check that the homing sensor == 1
        engage brake
        */
    if (retractStep == 0 && (readEncoder() >= 5.5))
    {
        winchMotor->Set(-kHangWinchSlowSpeed);
        std::cout << "slow" << '\n';
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
    else if ((fabs(readEncoder() - kEncoderSlowHeight) <= kEncoderTolerance) && (retractStep == 3))
    {
        winchMotor->Set(-kHangWinchSlowSpeed); 
        std::cout << "slow slow" << '\n';
        retractStep++;
        
    }
    else if (retractStep == 4)
    {
        if (fabs(readEncoder()- kEncoderSlowerHeight) <= kEncoderTolerance)
        {
            winchMotor->Set(-kHangWinchSlowerSpeed);
            std::cout << "slowslowslow" << '\n';
        }
        else if(homeSensor.Get() == 1){
            winchMotor->Set(0);
            step++;
        }
    }
}

void Hang::reversePivot()
{ 
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
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
    if (readEncoder() <= kBrokenExtendMax && readEncoder() >= kEncoderMin)
    {
        disengageBrake();
    }
    else if (readEncoder() >= 20){
        engageBrake();
    }
}

void Hang::windUpString()
{
    stringServo.Set(kServoBackward);
        if(hangTimer.Get().value() >= 2){
            stringServo.Set(kServoStopped);
            step++;
        }
}

void Hang::unwindString()
{
    stringServo.Set(kServoForward);
        if(hangTimer.Get().value() >= 2){
            stringServo.Set(kServoStopped);
            step++;
        }
}

void Hang::brokenRetract(){

    winchMotor->Set(kHangWinchSpeed);
    brokenStep++;

    if (brokenStep == 1){
        if (readEncoder() == hangBrokenSlowHeight){
            winchMotor->Set(kHangWinchSlowSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 2){
        if (readEncoder() == hangBrokenSlowerHeight){
            winchMotor->Set(kHangWinchSlowerSpeed);
            brokenStep++;
        }
    }
    else if (brokenStep == 3)
    {
        if (readEncoder() <= kEncoderMin)
        {
            winchMotor->Set(0);
        }
    }
}

void Hang::brokenExtendALittle()
{
    if (readEncoder() <= kBrokenExtendMax && readEncoder() >= kEncoderMin)
    {
        double currentEncoderValue = readEncoder();
        disengageBrake();
        if (readEncoder() - currentEncoderValue == kExtendDifference){
            engageBrake();
        }
        else if (readEncoder() - currentEncoderValue <= kBrokenExtendMax + 1)
        {
        }
        else{
            engageBrake();
        }
    }
}

void Hang::commandAuto()
{   
    switch(targetStage)
    {
        case STOP: 
            break;
        case NOT_ON_BAR:
            targetStage = MID;
            stepDone = false;
            break;
        case MID:
            if(stepDone == true)
            {
                targetStage = HIGH_TRAVERSAL;
                step = 0;
                stepDone = false;
            }
            break;
        case HIGH_TRAVERSAL:
            if(stepDone == true)
            {   
                hangBar++;
                targetStage = HIGH_TRAVERSAL;
                step = 0;
            }
            break;
    }
}

void Hang::commandManual(Manual manualCommands){
    isDone = false;
    manual = manualCommands;
}

double Hang::readEncoder(){
    #ifdef TEST_BOARD
    return winchEncoder.Get()/256.0;
    #else
    return winchMotor->GetAlternateEncoder();
    #endif
}

void Hang::resetEncoder(){
    #ifdef TEST_BOARD
        winchEncoder.Reset();
    #else
    //something
    winchMotor->SetAlternateEncoder(0);
    #endif
}