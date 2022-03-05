#include "Hang.h"
#include "string"
#include "iostream"


// hang retract speeds
//kHangWinchSpeed is for the initial retracting, going from fully extended to partially retracted
const double kHangWinchSpeed = .8;
//hangWinchSlowSpeed is for the next step, slowing down the arms
const double kHangWinchSlowSpeed = kHangWinchSpeed * .5;
//hangWinchSlowerSpeed slows the arms even more
const double kHangWinchSlowerSpeed = kHangWinchSlowSpeed * .5;
//difference between the current encoder value and what it was before the arms extended a little to get off the bar
const double kExtendDifference = 30;
//self explanatory
const double kRachetAndPawlBackdriveSpeed = -.1;

// timer times
const double kPivotTime = .5;
const double kReversePivotTime = .2;

//encoder constants
//minimum the encoder will go
const double kEncoderMin = .1;//-2.75;
//safety for extending - make sure that the arms are retracted enough before releasing the brake
const double kBrokenExtendMax = .15;
//max height
const double kEncoderMax = 6;//13.64;
const double kEncoderNearMax = 5.5;//13.1;
//height corresponding to kHangWinchSlowSpeed
const double kEncoderSlowHeight = kEncoderMax*.4;
//height corresponding to kHangWinchSlowerSpeed
const double kEncoderSlowerHeight = kEncoderSlowerHeight*.5;
//tolerance for maximum retract
const double kEncoderTolerance = .1;
//extendALittle encoder
const double kExtendALittleDistance = .5;//2;
//disengage brake
const double kDisengageBrake = -.2;//-11/360.0;
//hi ishan
//servo speed constants
const double kServoBackward = 0;
const double kServoForward = 1;
const double kServoStopped = .5;

const double kPawlForward = 0;//1;
const double kPawlReverse = 1;//0;

const double kDrumServoTime = 2;

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
    stringServoRight.Set(kServoStopped);
    ratchetServo.Set(kPawlForward);
    autoDone = false;
    manual = NOT;
    stepDone = true;
    currentEncoderValue = 0;
    manualStep = 0;
    highOrTraversal = 0;

}

void Hang::sendFeedback(){
    std::string targetString = "";
    switch (targetStage){
        case MID:
            targetString = "extending to mid";
            break;
        case MID_2:
            targetString = "retracting onto mid bar";
            break;
        case HIGH_TRAVERSAL:
            if(highOrTraversal == 0)
            {
                targetString = "going to/on high";
            }
            else if(highOrTraversal >= 2){
                targetString = "going to/on traversal";
            }
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
    std::string targetManualString = "";
    switch (manual){
        case EXTEND:
            targetManualString = "extending";
            break; 
        case EXTEND_A_LITTLE:
            targetManualString = "extending a little";
            break;
        case PIVOT_IN:
            targetManualString = "pivoting in";
            break;
        case PIVOT_OUT:
            targetManualString = "pivot out";
            break;
        case REVERSE_PIVOT:
            targetManualString = "reverse pivoting";
            break;
        case ENGAGE_BRAKE:
            targetManualString = "engaging brake";
            break;
        case DISENGAGE_BRAKE:
            targetManualString = "disengaging brake";
            break;
        case PULL_STRING:
            targetManualString = "pulling tring on servo";
            break;
        case UNWIND_STRING:
            targetManualString = "unwinding string on servo";
            break;
        case RETRACT:
            targetManualString = "retracting";
            break;
        case NOT:
            targetManualString = "nothing";
            break;
    }


    Feedback::sendDouble("Hang", "encoder value", readEncoder());
    Feedback::sendDouble("Hang", "winch motor speed", winchMotor->Get());
    Feedback::sendDouble("Hang", "Max extending arm height", hangMaxHeight);
    Feedback::sendDouble("Hang", "current step", step);
    Feedback::sendBoolean("Hang", "Home sensor value", homeSensor.Get());
    Feedback::sendString("Hang", "stage robot is going to", targetString.c_str());
    Feedback::sendString("Hang", "action robot is doing in manual", targetManualString.c_str());
    Feedback::sendDouble("Hang", "extend step value", extendStep);
    Feedback::sendDouble("Hang", "retract step", retractStep);
    Feedback::sendBoolean("Hang", "is auto hang done", autoDone);
    Feedback::sendDouble("Hang", "broken step value - not useful most of the time", brokenStep);
    Feedback::sendDouble("thunderdashboard", "hang_bar", hangBar);
    Feedback::sendDouble("thunderdashboard", "hang_status", hangStatus);
    Feedback::sendDouble("Hang", "drum servo speed", stringServoRight.Get());
    Feedback::sendDouble("Hang", "position of rachet and pawl servo", ratchetServo.Get());
    Feedback::sendBoolean("Hang", "broken test value", test);
    Feedback::sendDouble("Hang", "manual step", manualStep);
    Feedback::sendDouble("Hang", "timer", hangTimer.Get().value());
    Feedback::sendDouble("Hang", "disengage brake start", disengageBrakeStart);
}

void Hang::process(){
//std::cout << targetStage << "," << manual << '\n';
if(autoDone == false && manual != NOT)
    {
        switch (manual)
        {
            case EXTEND:
                extend();
                //brokenExtend();  
                break;
            case EXTEND_A_LITTLE:
                //brokenExtendALittle();
                extendALittle();
                break;
            case RETRACT:
                retract();
                //brokenRetract();
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
                disengageBrake();
                break;
            case DISENGAGE_BRAKE:
                disengageBrake();
                break;
            case NOT:
                manualStep = 0;
                break;
        }
    }
    else{
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
                pivot(true);
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
                stepDone=true;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
        case MID_2:
            if (step == 0)
            {
                retractStep = 0;
                extendStep = 0;
                step++;
                std::cout << "reset" << '\n';
            }
            else if(step == 1)
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
                std::cout << targetStage << '\n';
                if(highOrTraversal == 1){
                    highOrTraversal++;
                }
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
                pivot(false);
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
                pivot(true);
                step++;
                hangTimer.Reset();
                std::cout << "pivot" << '\n';
            }
            else if(step == 11)
            {
                windUpString();
                autoDone = true;
                stepDone = true;
                highOrTraversal++;
                extendStep = 0;
                std::cout << "windUpString" << '\n';
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
    }
    }
}

void Hang::manualReset()
{
    manualStep = 0;
    brokenStep = 0;
    step = 0;
    extendStep = 0;
}

void Hang::pivot(bool armsForward)
{ 
    isDone = true;
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    if(armsForward == true){
        //if true, arms go forwards
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
    if (extendStep == 0)
    {
        disengageBrakeStart = readEncoder();
        extendStep++;
    }
    else if(extendStep == 1 && disengageBrake()){
            extendStep++;
    }
    else if (extendStep == 2){
        #ifdef TEST_BOARD
        winchMotor->Set(.5);
        #endif
        extendStep++;
    }
    else if (extendStep == 3 && readEncoder() >= kEncoderNearMax)
    {
        step++;
        manualStep++;
        currentManualState = EXTEND;
        #ifdef TEST_BOARD
        winchMotor->Set(0);
        #endif
    }
}

void Hang::extendALittle()
{
    if ((homeSensor.Get() == 1 || readEncoder() <= kEncoderMin) && extendStep == 0){
        disengageBrakeStart = readEncoder();
        extendStep++;
    }
    else if(extendStep == 1 && disengageBrake()){
            extendStep++;
    }
    else if(extendStep == 2){
        #ifdef TEST_BOARD
        winchMotor->Set(0.6);
        #endif
        extendStep++;
    }
    else if (readEncoder() >= kExtendALittleDistance && extendStep == 3)
    {
        winchMotor->Set(0);
        engageBrake();
        //std::cout << "completed" << '\n';
        extendStep++;
    }
    else if(extendStep == 4){
        step++;
        manualStep++;
        currentManualState = EXTEND_A_LITTLE;
    }
}

void Hang::retract()
{
    double currentEncoderValue = readEncoder();
    if(currentEncoderValue >= kEncoderSlowHeight){
        winchMotor->Set(-kHangWinchSpeed);
    }
    else if(currentEncoderValue <= kEncoderSlowHeight && currentEncoderValue >= kEncoderSlowerHeight){
        winchMotor->Set(-kHangWinchSlowSpeed); 
    }
    else if(currentEncoderValue <= kEncoderSlowerHeight && currentEncoderValue >= kEncoderMin){
        winchMotor->Set(-kHangWinchSlowerSpeed);
    }
    else if(homeSensor.Get() == 1 || currentEncoderValue <= kEncoderMin){
        winchMotor->Set(0);
        manualStep++;
        step++;
        currentManualState = RETRACT;
    } 
}

void Hang::reversePivot()
{ 
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
    if (hangTimer.Get().value() >= kReversePivotTime)
    {
        step++;
    }
}

void Hang::engageBrake()
{                  
    ratchetServo.Set(kPawlForward);
    step++;
} // HI ISHAN


bool Hang::disengageBrake()
{  
    std::cout << "read encoder:" << readEncoder() << "dis+kDis" << disengageBrakeStart+kDisengageBrake << "\n";
    if(readEncoder() <= disengageBrakeStart+kDisengageBrake){
        std::cout << "encoder read successfully\n";
        winchMotor->Set(0);
        std::cout << "disengaging brake if\n";
        return true;
    }
    else{
        std::cout << "disengaging brake else\n";
        winchMotor->Set(kRachetAndPawlBackdriveSpeed);
        ratchetServo.Set(kPawlReverse);
        return false;
    }
    //make sure pawl is disengaged, pull servo
    //get direction down
}



void Hang::windUpString()
{
        if(hangTimer.Get().value() >= kDrumServoTime){
            stringServoRight.Set(kServoStopped);
            step++;
            hangTimer.Stop();
        }
        else {
            stringServoRight.Set(kServoBackward);
        }
}

void Hang::unwindString()
{
    if(hangTimer.Get().value() >= kDrumServoTime){
            stringServoRight.Set(kServoStopped);
            step++;
            hangTimer.Stop();
        }
    else {
        stringServoRight.Set(kServoForward);
    }
}

void Hang::commandAuto()
{   
    switch(targetStage)
    {
        case STOP: 
            break;
        case NOT_ON_BAR:
            hangBar++;
            targetStage = MID;
            stepDone = false;
            break;
        case MID:
            if(stepDone == true)
            {
                targetStage = MID_2;
                step = 0;
                stepDone = false;
            }
            break;
        case MID_2:
            if(stepDone == true)
            {   
                hangBar++;
                targetStage = HIGH_TRAVERSAL;
                step = 0;
                stepDone = false;
            }
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
    if(manual != currentManualState){
        manualStep = 0;
        brokenStep = 0;
        step = 0;
        extendStep = 0;
        retractStep = 0;
        hangTimer.Reset();
        hangTimer.Start();
    }
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