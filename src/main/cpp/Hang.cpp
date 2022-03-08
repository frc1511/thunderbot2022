#include "Hang.h"
#include "string"
#include "iostream"


// hang retract speeds
//kHangWinchSpeed is for the initial retracting, going from fully extended to partially retracted
const double kHangWinchSpeed = 1;//1;
//hangWinchSlowSpeed is for the next step, slowing down the arms
const double kHangWinchSlowSpeed = 1;//1;
//hangWinchSlowerSpeed slows the arms even more
const double kHangWinchSlowerSpeed = .75;//.75;
//self explanatory
const double kRachetAndPawlBackdriveSpeed = .3;
const double kDriveDownSpeed = .1;
const double kRetractLimitedSpeed = .2;
const double kExtendBackdrive = .1;

//encoder constants
//minimum the encoder will go
const double kEncoderMin = -3.23;//-2.99121;
//near max
const double kEncoderNearMax = 13.1;
//max height
const double kEncoderMax = 13.64;
//height corresponding to kHangWinchSlowSpeed
const double kEncoderSlowHeight = kEncoderMax*.4;
//height corresponding to kHangWinchSlowerSpeed
const double kEncoderSlowerHeight = .5;
//extendALittle encoder distance
const double kEncoderSmallExtension = 2;
//disengage brake
const double kEncoderDisengageBrake = -11/360.0;
//disengage brake height at the end of retractForHigh so we can shift to static arms
const double kEncoderDisenageInRetract = -2.23; //75% of the way to kEncoderMin from kEncoderSlowerHeight
//hi ishan
//servo speed constants
const double kServoBackward = 1;
const double kServoForward = 0;
const double kServoStopped = .5;

const double kPawlForward = .8;
const double kPawlReverse = .57;

const double kStringServoTime = 1.5;
const double kShiftToStaticArmTime = 5;

Hang::Hang() : winchMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::Hang)) {
    configureMotor();
    winchMotor->ConfigAlternateEncoder(2048);
    resetEncoder();
}
Hang::~Hang() {}

void Hang::doPersistentConfiguration(){
    configureMotor();
    winchMotor->BurnFlash();
}

void Hang::resetToMode(MatchMode mode){
    stringServoRight.Set(kServoStopped);
    stringServoLeft.Set(kServoStopped);
    ratchetServo.Set(kPawlForward);
    winchMotor->Set(0);
    //pistons are the opposite of what you think
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
    if(mode != MatchMode::MODE_DISABLED)
    {
        resetEncoder();
        step = 0;
        targetStage = NOT_ON_BAR;
        hangTimer.Reset();
        retractStep = 0;
        extendStep = 0;
        brokenStep = 0;
        hangBar = 0;
        autoDone = false;
        manual = NOT;
        stepDone = true;
        currentEncoderValue = 0;
        manualStep = 0;
        highOrTraversal = 0;
        currentManualState = NOT;
        disengageBrakeDone = false;
        retractDone = false;
        retractCurrentIncrease = false;
    }
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
            if(hangBar == 2)
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
        case DRIVE_DOWN:
            targetManualString = "drive down";
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
    Feedback::sendDouble("Hang", "hang bar", hangBar);
    Feedback::sendDouble("thunderdashboard", "hang_status", hangStatus);
    Feedback::sendDouble("Hang", "manual step", manualStep);
    Feedback::sendDouble("Hang", "timer", hangTimer.Get().value());
    Feedback::sendDouble("Hang", "disengage brake start", disengageBrakeStart);
    Feedback::sendDouble("Hang", "winch motor current", winchMotor->GetOutputCurrent());
    Feedback::sendDouble("Hang", "stickyfaults", winchMotor->GetStickyFaults());
    Feedback::sendDouble("Hang", "faults", winchMotor->GetFaults());
    Feedback::sendDouble("Hang", "winch motor temp", winchMotor->GetMotorTemperatureFarenheit());
}

void Hang::process(){
//std::cout << targetStage << "," << manual << '\n';
if(autoDone == false && manual != NOT)
    {
        hangBar = 0;
        switch (manual)
        {
            case EXTEND:
                extend();
                //brokenExtend();  
                break;
            case EXTEND_A_LITTLE:
                //brokenExtendALittle();
                //extendALittle();
                break;
            case RETRACT:
                if(retractDone == false){
                    retract();
                }
                break;
            case PULL_STRING:
                windUpString();
                break;
            case UNWIND_STRING:
                unwindString();
                break;
            case PIVOT_OUT:
                pivot(true);
                break;
            case PIVOT_IN:
                pivot(false);
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
            case DRIVE_DOWN:
                if(homeSensor.Get() == 1){
                    winchMotor->Set(0);
                }
                else {
                    winchMotor->Set(-kDriveDownSpeed);
                }
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
                std::cout << "reset" << '\n';
            }
            else if(step == 1)
            {   
                extend();
            }
            else if (step == 2)
            {
                engageBrake();
                step++;
                winchMotor->Set(0);
                std::cout << "engage brake" << '\n';
                stepDone=true;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
        case MID_2:
            if(step == 0)
            {
                hangTimer.Reset();
                hangTimer.Start();
                stringServoLeft.Set(kServoForward);
                stringServoRight.Set(kServoBackward);
                step++;
            }
            else if (step == 1)
            {   
                stepDone = false;
                retractStep = 0;
                extendStep = 0;
                if(hangTimer.Get().value() >= .5)
                {
                    step++;
                }
                retractCurrentIncrease = false;
                std::cout << "reset" << '\n';
            }
            else if(step == 2)
            {
                retract(); //retractForHigh()
                stringServoRight.Set(kServoStopped);
                stringServoLeft.Set(kServoStopped);
                //std::cout << "retract" << '\n';
            }
            else if(step == 3)
            {
                //extendALittle();
                stepDone = true;
                
            }
            else
            {
                winchMotor->Set(0);
            }
            if(step == 4)
            {
                stepDone = true;
            }
            break;
        case HIGH_TRAVERSAL:
            if (step == 0)
            {
                std::cout << targetStage << '\n';
                if(highOrTraversal == 1){
                    highOrTraversal++;
                }
                stepDone = false;
                autoDone = false;
                step++;
            }/*
            else if (step == 1)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                std::cout << "resetting" << '\n';
            }
            else if(step == 2)
            {
                //both functions add 1 to step
                unwindString();
                pivot(false);
                std::cout << "pivot" << '\n';
            }
            else if (step == 4)
            {
                extendStep = 0;
                step++;
                std::cout << "resetting" << '\n';
            }
            else if (step == 5)
            {
                extend();
                std::cout << "extending" << '\n';
            }
            else if (step == 6)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                std::cout << "resetting" << '\n';
            }
            else if (step == 7)
            {
                reversePivot();
                std::cout << "reverse pivot" << '\n';
            }
            else if (step == 8)
            {
                engageBrake();
                step++;
                std::cout << "engage brake" << '\n';
            }
            else if (step == 9)
            {
                retractStep = 0;
                retractCurrentIncrease = false;
                step++;
                std::cout << "reset" << '\n';
            }
            else if(step == 10)
            {
                retract();
                std::cout << "retract" << '\n';
            }
            else if(step == 11)
            {
                pivot(true);
                hangTimer.Reset();
                hangTimer.Start();
                std::cout << "pivot" << '\n';
            }
            else if(step == 12)
            {
                windUpString();
                autoDone = true;
                stepDone = true;
                highOrTraversal++;
                extendStep = 0;
                std::cout << "windUpString" << '\n';
            }*/
            else
            {
                winchMotor->Set(0);
            }
            break;
    }
    }
}

void Hang::configureMotor(){
    winchMotor->RestoreFactoryDefaults();
    winchMotor->SetInverted(true);
    winchMotor->SetSmartCurrentLimit(180);
    winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
}

void Hang::pivot(bool armsForward)
{ 
    // toggles hang pivot piston to rotate extending arms forwards or backwards
    if(armsForward == true){
        //if true, arms go forwards
        hangPivot1.Set(frc::DoubleSolenoid::kForward);
        hangPivot2.Set(frc::DoubleSolenoid::kForward);
        step++;
    }
    else{
        hangPivot1.Set(frc::DoubleSolenoid::kReverse);
        hangPivot2.Set(frc::DoubleSolenoid::kReverse);
        step++;
    }
}

void Hang::extend()
{
    if (extendStep == 0)
    {
        disengageBrakeStart = readEncoder();
        disengageBrakeDone = false;
        hangTimer.Reset();
        hangTimer.Start();
        extendStep++;
    }
    else if(extendStep == 1 && disengageBrake()){
           // winchMotor->Set(kExtendBackdrive);
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
        winchMotor->Set(0);
        #ifdef TEST_BOARD
        winchMotor->Set(0);
        #endif
    }
}

void Hang::extendALittle()
{
    if(extendStep == 0){
        #ifdef TEST_BOARD
        winchMotor->Set(0.6);
        #endif
        extendStep++;
    }
    else if (readEncoder() >= kEncoderSmallExtension && extendStep == 1)
    {
        winchMotor->Set(0);
        engageBrake();
        //std::cout << "completed" << '\n';
        extendStep++;
    }
    else if(extendStep == 2){
        step++;
        manualStep++;
    }
}

void Hang::retract()
{
    ratchetServo.Set(kPawlForward);
    double currentEncoderValue = readEncoder();
    if(homeSensor.Get() == 1 || currentEncoderValue <= kEncoderMin){
        winchMotor->Set(0);
        manualStep++;
        step++;
        retractDone = true;
    }
    else if(retractCurrentIncrease == false){
        winchMotor->Set(-kRetractLimitedSpeed);
        if(winchMotor->GetOutputCurrent() >= 35)
        {
            retractCurrentIncrease = true; 
        }
    }
    else if(retractCurrentIncrease == true) {
        if(currentEncoderValue >= kEncoderSlowHeight){
            winchMotor->Set(-kHangWinchSpeed);
        }
        else if(currentEncoderValue <= kEncoderSlowHeight && currentEncoderValue >= kEncoderSlowerHeight){
            winchMotor->Set(-kHangWinchSlowSpeed); 
        }
        else if(currentEncoderValue <= kEncoderSlowerHeight && currentEncoderValue >= kEncoderMin){
            winchMotor->Set(-kHangWinchSlowerSpeed);
        }
    }
}

void Hang::reversePivot()
{ 
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
    step++;
}

void Hang::engageBrake()
{                  
    ratchetServo.Set(kPawlForward);

} // HI ISHAN


bool Hang::disengageBrake()
{  
    std::cout << "read encoder:" << readEncoder() << "dis+kDis" << disengageBrakeStart+kEncoderDisengageBrake << "\n";
    if(readEncoder() <= disengageBrakeStart+kEncoderDisengageBrake){
        winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        std::cout << "encoder read successfully\n";
        winchMotor->Set(0);
        std::cout << "disengaging brake if\n";
        disengageBrakeDone = true;
        return true;
    }
    else if(disengageBrakeDone == false){
        std::cout << "disengaging brake else\n";
        ratchetServo.Set(kPawlReverse);
        if(hangTimer.Get().value() >= .5)
        {
            winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
            winchMotor->Set(-kRachetAndPawlBackdriveSpeed);
        }
        return false;
    }
    return false;
    //make sure pawl is disengaged, pull servo
    //get direction down
}



void Hang::windUpString()
{
        if(hangTimer.Get().value() >= kStringServoTime){
            stringServoRight.Set(kServoStopped);
            stringServoLeft.Set(kServoStopped);
            step++;
            hangTimer.Stop();
        }
        else {
            stringServoRight.Set(kServoBackward);
            stringServoLeft.Set(kServoForward);
        }
}

void Hang::unwindString()
{
    if(hangTimer.Get().value() >= kStringServoTime){
            stringServoRight.Set(kServoStopped);
            stringServoLeft.Set(kServoStopped);
            step++;
            hangTimer.Stop();
        }
    else {
        stringServoRight.Set(kServoForward);
        stringServoLeft.Set(kServoBackward);
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
               // targetStage = HIGH_TRAVERSAL;
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
    manual = manualCommands;
    if(manual != currentManualState){
        manualStep = 0;
        brokenStep = 0;
        step = 0;
        extendStep = 0;
        retractStep = 0;
        hangTimer.Reset();
        hangTimer.Start();
        disengageBrakeDone = false;
        retractDone = false;
        disengageBrakeStart = readEncoder();
        retractCurrentIncrease = false;
    }
    currentManualState = manual;
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


void Hang::retractForHigh()
{
    double currentEncoderValue = readEncoder();
    if(homeSensor.Get() == 1 || currentEncoderValue <= kEncoderMin){
        winchMotor->Set(0);
        retractDone = true;
        hangTimer.Reset();
        hangTimer.Start();
    }
    else if(hangTimer.Get().value() >= kShiftToStaticArmTime)
    {
        winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        manualStep++;
        step++;
    }
    else if(retractCurrentIncrease == false){
        winchMotor->Set(-kRetractLimitedSpeed);
        if(winchMotor->GetOutputCurrent() >= 35)
        {
            retractCurrentIncrease = true;
        }
    }
    else if(retractCurrentIncrease == true && retractDone == false) {
        if(currentEncoderValue >= kEncoderSlowHeight){
            winchMotor->Set(-kHangWinchSpeed);
            ratchetServo.Set(kPawlForward);
        }
        else if(currentEncoderValue <= kEncoderSlowHeight && currentEncoderValue >= kEncoderSlowerHeight){
            winchMotor->Set(-kHangWinchSlowSpeed); 
        }
        else if(currentEncoderValue <= kEncoderSlowerHeight && currentEncoderValue >= kEncoderDisenageInRetract){
            winchMotor->Set(-kHangWinchSlowerSpeed);
        }
        else if(currentEncoderValue<=kEncoderDisenageInRetract && currentEncoderValue >= kEncoderMin){
            winchMotor->Set(-kHangWinchSlowerSpeed);
            ratchetServo.Set(kPawlReverse);
            winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
        }
    }
}