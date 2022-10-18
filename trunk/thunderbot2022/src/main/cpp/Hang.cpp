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
const double kExtendBackdrive = .6;
const double kExtendSlowerBackdrive = .2;
const double kExtendLowSpeed = .2;

//encoder constants
//minimum the encoder will go
const double kEncoderMin = -3.23;//-2.99121;
//near max
const double kEncoderMidHeight = 11.497; // max is really 13.1;
const double kEncoder70percentMid = kEncoderMidHeight*.7;
//max height
const double kEncoderMax = 14.65; // max is really 13.64;
const double kEncoder70percentMax = kEncoderMax * .7;
const double kEncoderHalfRetracted =  5.5;//6.25;//8.94;
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
//low bar max
const double kEncoderLowHeight = 2.7397;
const double kEncoder70percentLow = kEncoderLowHeight*.7;
//hi ishan
//servo speed constants
const double kServoBackward = 1;
const double kServoForward = 0;
const double kServoStopped = .5;

const double kPawlForward = .8;
const double kPawlReverse = .57;

const double kStringServoTime = 3;
const double kShiftToStaticArmTime = 1.5;
const int kDisengageBrakeTime = .5;
const double kSwingToStaticArmsTime = 2.5;

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
    // stringServoRight.Set(kServoStopped);
    // stringServoLeft.Set(kServoStopped);
    ratchetServo.Set(kPawlForward);
    winchMotor->Set(0);
    teleopOrNo = false;
    //pistons are the opposite of what you think
    hangPivot1.Set(frc::DoubleSolenoid::Value::kForward);
    hangPivot2.Set(frc::DoubleSolenoid::Value::kReverse);
    if(mode != MatchMode::MODE_DISABLED)
    {
        resetEncoder();
        step = 0;
        targetStage = NOT_ON_BAR;
        extendLevel = MID_HEIGHT;
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
        isLowHeight = false;
        goingForHigh = false;
        highDone = false;
        pauseEnabled = false;
        pauseDisabled = false;
        teleopOrNo = true;
        currentTimer.Reset();
        currentTimer.Start();
        // std::ofstream HangCurrentLogs("/home/lvuser/hangCurrentLogs.txt");
        // HangCurrentLogs << "";
        // HangCurrentLogs.close();
        haveWeTraversed = false;
    }
}

void Hang::sendFeedback(){
    if(teleopOrNo){
        currentLog();
    }
    std::string targetString = "";
    switch (targetStage){
        case LOW:
            targetString = "extending to low bar";
            break;
        case LOW_2:
            targetString = "retracting onto low bar";
            break;
        case MID:
            targetString = "extending to mid bar";
            break;
        case MID_2:
            targetString = "retracting onto mid bar";
            break;
        case HIGH_TRAVERSAL:
            if(hangBar == 2)
            {
                targetString = "going to/on high bar";
            }
            else if(haveWeTraversed){
                targetString = "going to/on traversal bar";
            }
            else{
                targetString = "we are past traversal THIS IS BAD";
            }
            break;
        case HIGH_TRAVERSAL_2:
            targetString = "pulling onto high/traversal bar";
            break;
        case NOT_ON_BAR:
            targetString = "not currently on a bar";
            break;
        case PAUSE:
            targetString = "stopped";
            break;
    }
    std::string targetHeight = "";
    switch (extendLevel){
        case LOW_HEIGHT:
            targetHeight = "low bar";
            break;
        case MID_HEIGHT:
            targetHeight = "Mid bar";
            break;
        case HIGH_TRAVERSAL_HEIGHT:
            targetHeight = "high/traversal";
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

    if(manual != NOT)
    {
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
    }

    Feedback::sendBoolean("Hang", "step done", stepDone);
    Feedback::sendBoolean("Hang", "retract done", retractDone);
    Feedback::sendBoolean("Hang", "going for high bar", goingForHigh);
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
    Feedback::sendDouble("Hang", "timer for current tracking", currentTimer.Get().value());
    Feedback::sendDouble("Hang", "disengage brake start", disengageBrakeStart);
    Feedback::sendDouble("Hang", "winch motor current", winchMotor->GetOutputCurrent());
    Feedback::sendDouble("Hang", "stickyfaults", winchMotor->GetStickyFaults());
    Feedback::sendDouble("Hang", "faults", winchMotor->GetFaults());
    Feedback::sendDouble("Hang", "winch motor temp", winchMotor->GetMotorTemperatureFarenheit());
    Feedback::sendBoolean("Hang", "is high done", highDone);
    Feedback::sendString("Hang", "ideal arm height", targetHeight.c_str());
    Feedback::sendBoolean("Hang", "if we have traversed", haveWeTraversed);
    Feedback::sendDouble("Hang", "highOrTraversal variable", highOrTraversal);
}

void Hang::process(){
////std::cout << targetStage << "," << manual << '\n';
if(autoDone == false && manual != NOT)
    {
        hangBar = 0;
        switch (manual)
        {
            case EXTEND:
                extendLevel = HIGH_TRAVERSAL_HEIGHT;
                extend();
                break;
            case EXTEND_A_LITTLE:
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
            case DISENGAGE_BRAKE:
                disengageBrake();
                break;
            case ENGAGE_BRAKE:
                engageBrake();
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
        case PAUSE:
            ratchetServo.Set(kPawlForward);
            //// stringServoLeft.Set(kServoStopped);
            //// stringServoRight.Set(kServoStopped);
            winchMotor->Set(0);
            break;
        case NOT_ON_BAR:
            winchMotor->Set(0);
            //ratchetServo.Set(kPawlForward);
            break;
        case LOW:
            if (step == 0)
            {   
                completedTargetStage = NOT_ON_BAR;
                pivot(true);
                extendStep = 0;
                extendLevel = LOW_HEIGHT;
            }
            else if(step == 1)
            {   
                extend();
            }
            else if (step == 2)
            {
                engageBrake();
                completedTargetStage = LOW;
                winchMotor->Set(0);;
                stepDone=true;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
        case LOW_2:
            if(step == 0)
            {
                hangTimer.Reset();
                hangTimer.Start();
                //// stringServoLeft.Set(kServoForward);
                //// stringServoRight.Set(kServoBackward);
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
            }
            else if(step == 2){
                retract();
                //// stringServoRight.Set(kServoStopped);
                //// stringServoLeft.Set(kServoStopped);
            }
            else if(step == 3){
                stepDone = true;
                completedTargetStage = LOW_2;
            }
            break;
        case MID:
            if (step == 0)
            {   
                pivot(true);
                extendStep = 0;
                extendLevel = MID_HEIGHT;
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
                stepDone=true;
                completedTargetStage = MID;
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
                //// stringServoLeft.Set(kServoForward);
                //// stringServoRight.Set(kServoBackward);
                step++;
            }
            else if (step == 1)
            {   
                stepDone = false;
                retractStep = 0;
                extendStep = 0;
                retractCurrentIncrease = false;
                hangTimer.Stop();
                hangTimer.Reset();
                step++;
            }
            else if(step == 2){
                if(goingForHigh){
                    retractForHigh();
                }
                else{
                    retract();
                }
                //// stringServoRight.Set(kServoStopped);
                //// stringServoLeft.Set(kServoStopped);
            }
            else if((step == 3 && !goingForHigh) || (step == 4 && goingForHigh)){ // for normal mid it is step 3, for high bar mid it is 6
                stepDone = true;
                completedTargetStage = MID_2;

            }
            if(goingForHigh)
            {
                if(step == 3){
                    hangTimer.Reset();
                    hangTimer.Start();
                    step++;
                }
                /*else if(step == 4){
                    unwindString();
                }*/
            }
            break;
        case HIGH_TRAVERSAL:
            if (step == 0)
            {
                if(highOrTraversal == 1){
                    highOrTraversal++;
                    haveWeTraversed = true;
                }
                stepDone = false;
                autoDone = false;
                step++;      
            }
            else if (step == 1)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                //std::cout << "resetting" << '\n';
            }
            /*else if(step == 2)
            {
                unwindString();
            }*/
            else if(step == 2)
            {
                pivot(false);
                //std::cout << "pivot" << '\n';
            }
            else if (step == 3)
            {
                extendStep = 0;
                step++;
                extendLevel = HIGH_TRAVERSAL_HEIGHT;
                hangTimer.Reset();
                hangTimer.Start();
                //std::cout << "resetting" << '\n';
            }
            else if (step == 4)
            {
                extend();
                //std::cout << "extending" << '\n';
            }
            else if (step == 5)
            {
                hangTimer.Reset();
                hangTimer.Start();
                step++;
                //std::cout << "resetting" << '\n';
            }
            else if (step == 6)
            {
                reversePivot();
                //std::cout << "reverse pivot" << '\n';
                stepDone = true;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
        case HIGH_TRAVERSAL_2:
            if (step == 0)
            {
                engageBrake();
                step++;
                //std::cout << "engage brake" << '\n';
            }
            else if (step == 1)
            {
                retractStep = 0;
                retractCurrentIncrease = false;
                step++;
                //std::cout << "reset" << '\n';
            }
            else if(step == 2)
            {
                retractMaxToHalf();
                //std::cout << "retracting max to half" << '\n';
            }
            else if(step == 3)
            {
                pivot(true);
                hangTimer.Reset();
                hangTimer.Start();
                //std::cout << "pivoting" << '\n';
                
            }
            else if(step == 4)
            {
                //windUpString();
                retractCurrentIncrease = false;
                retractDone = false;
                //std::cout << "winding string" << '\n';
                if(hangTimer.Get().value() >= kSwingToStaticArmsTime){
                    step++;
                    hangTimer.Reset();
                    hangTimer.Start();
                }
            }
            else if(step == 5)
            {
                if(highDone == false){
                    retractHalfToStaticArms();
                    //std::cout << "going to static arms" << '\n';
                }
                else{
                    retractHalfToFull();
                }
                ////std::cout <<"retracting half to full" << "\n";
            }
            else if(step == 6){
                autoDone = true;
                stepDone = true;
                highDone = true;
                highOrTraversal++;
                step++;
                extendStep = 0;
            }
            else
            {
                winchMotor->Set(0);
            }
            break;
        }
    }

    // Hi calla!!
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
        winchMotor->SetIdleMode(ThunderSparkMax::COAST);
        disengageBrakeStart = readEncoder();
        disengageBrakeDone = false;
        hangTimer.Reset();
        hangTimer.Start();
        extendStep++;
    }
    else if(extendStep == 1 && disengageBrake()){
        std::cout << "extending\n";
        extendStep++;
    }
    else if(hangTimer.Get().value() >= kDisengageBrakeTime && extendStep == 2){
            switch (extendLevel){
                case LOW_HEIGHT:
                    winchMotor->Set(kExtendLowSpeed);
                    break;
                case MID_HEIGHT:
                    winchMotor->Set(kExtendBackdrive);
                    break;
                case HIGH_TRAVERSAL_HEIGHT:
                    winchMotor->Set(kExtendBackdrive);
                    break;
            }
            std::cout << "full speed\n";
            extendStep++;
    }
    else if (extendStep == 3){
        #ifdef TEST_BOARD
        winchMotor->Set(.5);
        #endif
        extendStep++;
    }
    else if(extendStep == 4){
        winchMotor->SetIdleMode(ThunderSparkMax::BRAKE);
        extendStep++;
    }
    else if (extendStep == 5) {
        switch (extendLevel){
            case LOW_HEIGHT:
                if(readEncoder() >= kEncoderLowHeight){
                    step++;
                    manualStep++;
                    winchMotor->Set(0);
                    winchMotor->SetIdleMode(ThunderSparkMax::COAST);
                }
                break;
            case MID_HEIGHT:
                if(readEncoder() >= kEncoderMidHeight){
                    step++;
                    manualStep++;
                    winchMotor->Set(0);
                    winchMotor->SetIdleMode(ThunderSparkMax::COAST);
                }
                else if(readEncoder() >= kEncoder70percentMid){
                    winchMotor->Set(kExtendSlowerBackdrive);
                }
                break;
            case HIGH_TRAVERSAL_HEIGHT:
                if(readEncoder() >= kEncoderMax){
                    step++;
                    manualStep++;
                    winchMotor->Set(0);
                    winchMotor->SetIdleMode(ThunderSparkMax::COAST);
                }
                else if(readEncoder() >= kEncoder70percentMax){
                    winchMotor->Set(kExtendSlowerBackdrive);
                }
                break;
        }
        #ifdef TEST_BOARD
        winchMotor->Set(0);
        #endif
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
    winchMotor->Set(0);

}

bool Hang::disengageBrake()
{  
    if(readEncoder() <= disengageBrakeStart+kEncoderDisengageBrake){
        winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        winchMotor->Set(0);
        disengageBrakeDone = true;
        return true;
    }
    else if(disengageBrakeDone == false){
        ratchetServo.Set(kPawlReverse);
        if(hangTimer.Get().value() >= .5)
        {
            winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
            winchMotor->Set(-kRachetAndPawlBackdriveSpeed);
        }
        return false;
    }
    return false;
}

void Hang::windUpString()
{
    if(hangTimer.Get().value() >= kStringServoTime){
        // stringServoRight.Set(kServoStopped);
        // stringServoLeft.Set(kServoStopped);
        step++;
        hangTimer.Stop();
    }
    else {
        // stringServoRight.Set(kServoBackward);
        // stringServoLeft.Set(kServoForward);
    }
}

void Hang::unwindString()
{
    if(hangTimer.Get().value() >= kStringServoTime){
            // stringServoRight.Set(kServoStopped);
            // stringServoLeft.Set(kServoStopped);
            step++;
            hangTimer.Stop();
        }
    else {
        // stringServoRight.Set(kServoForward);
        // stringServoLeft.Set(kServoBackward);
    }
}

void Hang::commandAuto()
{   
    switch(targetStage)
    {
        case PAUSE: 
            if(pauseDisabled == true){
                switch(completedTargetStage){
                    case PAUSE:
                        break;
                    case NOT_ON_BAR:
                        if(getIsLow() == true){
                            targetStage = LOW;
                        }
                        else{
                            targetStage = MID;
                        }
                        break;
                    case LOW:
                        targetStage = LOW_2;
                        break;
                    case LOW_2:
                        break;
                    case MID:
                        targetStage = MID_2;
                        break;
                    case MID_2:
                        targetStage = HIGH_TRAVERSAL;
                        break;
                    case HIGH_TRAVERSAL:
                        if(highDone == true){
                            targetStage = HIGH_TRAVERSAL;
                        }
                        else if(highDone == false){
                            targetStage = HIGH_TRAVERSAL;
                        }
                        break;
                    case HIGH_TRAVERSAL_2:
                        // bye warning
                        break;
                }
            }
            break;
        case NOT_ON_BAR:
            hangBar++;
            if(getIsLow() == true){
                targetStage = LOW;
            }
            else{
                targetStage = MID;
            }
            stepDone = false;
            break;
        case LOW: // going to pull up onto low bar
            if(stepDone == true)
            {
                targetStage = LOW_2;
                step = 0;
                stepDone = false;
            }
            break;
        case LOW_2:
            // do nothing
            stepDone = true;
            break;
        case MID: // going to pull up onto mid bar
            if(stepDone == true)
            {
                targetStage = MID_2;
                step = 0;
                stepDone = false;
            }
            break;
        case MID_2: // going to start high bar
            if(stepDone == true && goingForHigh == true)
            {   
                hangBar++;
                targetStage = HIGH_TRAVERSAL;
                step = 0;
                stepDone = false;
            }
            break;
        case HIGH_TRAVERSAL: // going to pull up onto high bar
            if(stepDone == true)
            {
                targetStage = HIGH_TRAVERSAL_2;
                step = 0;
                stepDone = false;
            }
            break;
        case HIGH_TRAVERSAL_2: // going to start trevorversial
            if(stepDone == true)
            {   
                hangBar++;
                if(haveWeTraversed == true){
                    targetStage = NOT_ON_BAR;
                }
                else{
                    targetStage = HIGH_TRAVERSAL;
                }
                targetStage = HIGH_TRAVERSAL;
                step = 0;
                stepDone = false;
            }
            break;
    }
}

void Hang::setCommandAutoOverride() {
    //overridingStep = override;
    winchMotor->Set(0);
    step++;                     
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
        stepDone = false;
        disengageBrakeStart = readEncoder();
        retractCurrentIncrease = false;
        winchMotor->Set(0);
        targetStage = NOT_ON_BAR;
    }
    currentManualState = manual;
}

void Hang::commandHeight(ExtendLevel extendLevelCommand){
    extendLevel = extendLevelCommand;
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
    winchMotor->SetAlternateEncoder(kEncoderMin);
    #endif
}


void Hang::retractForHigh()
{
    double currentEncoderValue = readEncoder();
    if(homeSensor.Get() == 1 || currentEncoderValue <= kEncoderMin){
        winchMotor->Set(0);
        //std::cout <<"finished" << '\n';
        retractDone = true;
        hangTimer.Reset();
        hangTimer.Start();
    }
    else if (readEncoder() >= kEncoderSmallExtension && retractDone == true)
    {
        engageBrake();
        step++;
    }
    else if(hangTimer.Get().value() >= kShiftToStaticArmTime)
    {
        winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        manualStep++;
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

void Hang::setIsLow(bool isLow){
    isLowHeight = isLow;
}
bool Hang::getIsLow(){
    return isLowHeight;
}

void Hang::setPause(bool paused, bool unpaused){
    pauseEnabled = paused;

    pauseDisabled = unpaused;
}

void Hang::setGoingForHigh(bool highOrNot){
    if(targetStage == NOT_ON_BAR)
    {
        goingForHigh = highOrNot;
    }
}

void Hang::retractMaxToHalf()
{
    ratchetServo.Set(kPawlForward);
    double currentEncoderValue = readEncoder();
    if(currentEncoderValue <= kEncoderHalfRetracted){
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
        if(currentEncoderValue >= kEncoderHalfRetracted){
            winchMotor->Set(-kHangWinchSpeed);
        }
    }
}

void Hang::retractHalfToFull()
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
        if(currentEncoderValue <= kEncoderHalfRetracted && currentEncoderValue >= kEncoderSlowerHeight){
            winchMotor->Set(-kHangWinchSpeed);
        }
    }
    else if(currentEncoderValue <= kEncoderSlowerHeight && currentEncoderValue >= kEncoderMin){
        winchMotor->Set(-kHangWinchSlowerSpeed);
    }
}

void Hang::retractHalfToStaticArms(){
    double currentEncoderValue = readEncoder();
    if(homeSensor.Get() == 1 || currentEncoderValue <= kEncoderMin){
        winchMotor->Set(0);
        //std::cout <<"finished" << '\n';
        retractDone = true;
        hangTimer.Reset();
        hangTimer.Start();
    }
    else if (readEncoder() >= kEncoderSmallExtension && retractDone == true)
    {
        engageBrake();
        step++;
    }
    else if(hangTimer.Get().value() >= kShiftToStaticArmTime && retractDone == true)
    {
        winchMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        manualStep++;
    }
    else if(retractCurrentIncrease == false){
        winchMotor->Set(-kRetractLimitedSpeed);
        if(winchMotor->GetOutputCurrent() >= 35)
        {
            retractCurrentIncrease = true;
        }
    }
    else if(retractCurrentIncrease == true && retractDone == false) {
        if(currentEncoderValue <= kEncoderHalfRetracted && currentEncoderValue >= kEncoderSlowerHeight){
            winchMotor->Set(-kHangWinchSpeed);
            ratchetServo.Set(kPawlForward);
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

void Hang::currentLog(){
    // std::ofstream HangCurrentLogs;
    // HangCurrentLogs.open("/home/lvuser/hangCurrentLogs.txt", std::fstream::app);
    // HangCurrentLogs << "time: " << std::to_string(currentTimer.Get().value()) << ", current: " << std::to_string(winchMotor->GetOutputCurrent()) << "\n";
    // HangCurrentLogs.close();
}

void hangBrokeDontMovePistonsBad(){
    //hangPivot1.Set();
    //hangPivot2.Set();
}