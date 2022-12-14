#include "Intake.h"

#define INTAKE_MAX_VOLTAGE 12
#define INTAKE_MAX_AMPERAGE 30

const double kSpeedStageOne = .7;         // used for intaking
const double kSpeedStageOneLimbo = .2;    // used for when waiting at stage one when intaking (.2) maybe turn it up to get the ball in further
const double kSpeedStageTwo = .9;         // used for shooting
const double kSpeedStageTwoSlow = .1;    // used for intaking into stage two
const double kReverseSpeedStageOne = -.4; // used for outtaking
const double kReverseSpeedStageTwo = -.3; // used for outtaking
const double kSpeedStageOneFix = .2;      // used for slowing cargo 2 after it passes first beam brake

const units::second_t kDebouncerTime = 50_ms; // used for making the ball count work

Intake::Intake() : intakeMotorStageOne(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage1)),
                   intakeMotorStageTwo(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage2))
{
    configureMotors();
    didAutoExist = false;
}

Intake::~Intake()
{
}

void Intake::configureMotors()
{
    intakeMotorStageOne->RestoreFactoryDefaults();
    intakeMotorStageTwo->RestoreFactoryDefaults();

    intakeMotorStageOne->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
    intakeMotorStageTwo->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);

    // Voltage limit.
    intakeMotorStageOne->EnableVoltageCompensation(INTAKE_MAX_VOLTAGE);
    intakeMotorStageOne->EnableVoltageCompensation(INTAKE_MAX_VOLTAGE);
    // Amperage limit.
    intakeMotorStageOne->SetSmartCurrentLimit(INTAKE_MAX_AMPERAGE);
    intakeMotorStageOne->SetSmartCurrentLimit(INTAKE_MAX_AMPERAGE);

    // Inverted!!! :D
    intakeMotorStageOne->SetInverted(true);
    intakeMotorStageTwo->SetInverted(true);
}

void Intake::doPersistentConfiguration()
{
    configureMotors();
    intakeMotorStageOne->BurnFlash();
    intakeMotorStageTwo->BurnFlash();
}

void Intake::resetToMode(MatchMode mode)
{

    if (mode != MatchMode::MODE_DISABLED)
    {
        intakeMotorStageOne->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
        intakeMotorStageTwo->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
    }
    else
    {
        intakeMotorStageOne->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        intakeMotorStageTwo->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
    }
    horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse);
    intakeMotorStageOne->Set(0); // stage 1 stop
    intakeMotorStageTwo->Set(0);
    intakePosition = false;
    intakeSpeed = 0;
    if (mode == MatchMode::MODE_AUTO)
    {
        didAutoExist = true;
    }
    if (mode == MatchMode::MODE_AUTO || (mode == MatchMode::MODE_TELEOP && didAutoExist == false)) // combo (ish) reset so it works when just doing tele, and when doing both
    {
        if (checkSensor(&stageOneFlag))
        {
            ballCount = 1;
        }
        else
        {
            ballCount = 0;
        } // set the ball count to zero
        stageTwoOccupied = false;
        stageOneSensorPrevious = checkSensor(&stageOneFlag);
        stageTwoSensorPrevious = checkSensor(&stageTwoFlag);
        shooterSensorPrevious = checkSensor(&shooterFlag);
        // bool currentSensorHalfInput = checkSensor(&stageHalfFlag);

        // m_debouncer.Calculate(stageOneSensorPrevious);
    }
    countPending = false;
    stageOneFixing = false;
    currentState = STATE_STOP;
    targetDirection = NOTTAKE;
}

void Intake::ballCountIntake(bool currentSensorOneInput, bool currentSensorTwoInput)
{
     if (stageTwoSensorPrevious == true && currentSensorTwoInput == false)
    {
        stageTwoOccupied = true;
    }

    if (stageOneSensorPrevious == false && currentSensorOneInput == true)
    {
        // start timer and stuff
        countPending = true;
        ballCountFix.Reset();
        ballCountFix.Start();
    }
    
    if (countPending)
    {
        if (currentSensorOneInput == false)
        {
            // Thought we had a ball, but it went away
            // before our time period elapsed. Give up and start over
            countPending = false;
        }
        else if (ballCountFix.Get() >= kDebouncerTime)
        {
            ballCount++;
            countPending = false;
        }
    }

     if (stageTwoSensorPrevious == true && currentSensorTwoInput == false)
    {
        stageTwoOccupied = true;
    }
}

void Intake::ballCountOuttake(bool currentSensorOneInput, bool currentSensorTwoInput)
{

    if (stageOneSensorPrevious == true && currentSensorOneInput == false)
    {
        ballCount = 0;
        stageTwoOccupied = false;
    } // bc the sensor doesn't see a break in between the balls so if outtaking just set to zero bc unlikely to remove only 1 ball of 2

    if (stageTwoSensorPrevious == false && currentSensorTwoInput == true)
    {
        stageTwoOccupied = false;
    }
}
void Intake::stageOneFix(bool currentSensorHalfInput, bool currentSensorOneInput)
{
    if (stageTwoOccupied && currentSensorHalfInput == false && currentSensorOneInput == true)
    {
        stageOneFixing = true;
    }
    else
    {
        stageOneFixing = false;
    }
}

void Intake::switchStates() // command switches
{
    switch (targetDirection)
    {
    case INTAKE:
        if (ballCount == 2)
        {
            currentState = STATE_INTAKE_TWO_BALL;
        }
        else
        {
            /*if (countPending)
            { // ball possibly there....
                currentState = STATE_PRE_COUNT;
            }
            else
            { // else keep looking
                */
            currentState = STATE_INTAKE_ONE_BALL;
            //}
        }
        break;
    case OUTTAKE:
        currentState = STATE_OUTTAKE;
        break;
    case NOTTAKE:
        currentState = STATE_STOP;
        break;
    case MANUAL:
        currentState = STATE_MANUAL;
        break;
    case SHOOTING:
        if (stageTwoOccupied == false)
        {
            currentState = STATE_STOP;
        }
        else
        {
            currentState = STATE_SHOOT;
        }
        break;
    }
    if (countPending)
    { // ball possibly there....
        currentState = STATE_PRE_COUNT;
    }
    if (stageOneFixing)
    {
        currentState = STATE_INTAKE_BRING_BALL_IN;
    }

    /*if (targetDirection != INTAKE)
    {
        ballCountFix.Stop();
        ballCountFix.Reset();
        countPending = false;
    }*/
}
void Intake::process()
{
    States startState = currentState;
    // bool currentSensorOneInput = m_debouncer.Calculate(checkSensor(&stageOneFlag));
    do
    {   
        if(ballCount > 2){
            ballCount = 2;
        }
        startState = currentState;
        bool currentSensorOneInput = checkSensor(&stageOneFlag);
        bool currentSensorTwoInput = checkSensor(&stageTwoFlag);
        bool currentShooterSensorInput = checkSensor(&shooterFlag);
        bool currentSensorHalfInput = checkSensor(&stageHalfFlag);
        // the desired actions of the intake motors
        switch (currentState)
        { // extend piston to retract intake, retract piston to extend/deploy intake
        case STATE_INTAKE_ONE_BALL:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward); // deploy intake, forward is forward
            intakeMotorStageOne->Set(kSpeedStageOne);                   // stage 1 go
            if (stageTwoOccupied)
            {                                // ball in stage two
                intakeMotorStageTwo->Set(0); // stage 2 go, slow
            }
            else
            {
                intakeMotorStageTwo->Set(kSpeedStageTwoSlow);
            }
            ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
            stageOneFix(currentSensorHalfInput, currentSensorOneInput);
            switchStates();
            break;
        case STATE_PRE_COUNT:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward);
            intakeMotorStageOne->Set(kSpeedStageOneLimbo);
            if (stageTwoOccupied)
            {                                // ball in stage two
                intakeMotorStageTwo->Set(0); // stage 2 go, slow
            }
            else
            {
                intakeMotorStageTwo->Set(kSpeedStageTwoSlow);
            }
            ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
            stageOneFix(currentSensorHalfInput, currentSensorOneInput);
            switchStates();
            break;
        case STATE_INTAKE_BRING_BALL_IN:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kOff);
            intakeMotorStageOne->Set(kSpeedStageOneFix);
            ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
            stageOneFix(currentSensorHalfInput, currentSensorOneInput);
            switchStates();
            break;
        case STATE_INTAKE_TWO_BALL:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
            intakeMotorStageOne->Set(0);                                // stage 1 stop
            intakeMotorStageTwo->Set(0);                                // stage 2 stop
            ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
            stageOneFix(currentSensorHalfInput, currentSensorOneInput);
            switchStates();
            break;
        case STATE_OUTTAKE:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kOff); // change this to off // leave intake as is
            intakeMotorStageOne->Set(kReverseSpeedStageOne);        // stage 1 go reverse
            intakeMotorStageTwo->Set(kReverseSpeedStageTwo);        // stage 2 go reverse
            ballCountOuttake(currentSensorOneInput, currentSensorTwoInput);
            switchStates();
            break;
        case STATE_STOP:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
            if (stageTwoOccupied == true || ballCount < 1)              // if there is a ball in position (stage 2) or if there are no balls
            {
                intakeMotorStageOne->Set(0); // stage 1 stop
                intakeMotorStageTwo->Set(0); // stage 2 stop
            }
            else // if there is a ball not in position
            {
                intakeMotorStageOne->Set(kSpeedStageOne);     // stage 1 go
                intakeMotorStageTwo->Set(kSpeedStageTwoSlow); // stage 2 go, slow
            }
            ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
            stageOneFix(currentSensorHalfInput, currentSensorOneInput);
            switchStates();
            break;
        case STATE_SHOOT:
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
            if (stageTwoOccupied == true)
            {
                intakeMotorStageTwo->Set(kSpeedStageTwo); // stage 2 go
                intakeMotorStageOne->Set(0);              // stage 1 stops
            }
            break;
        case STATE_WAIT_AFTER_SHOT:
            if(shotWaitTimer.HasElapsed(.5_s)){
                currentState = STATE_STOP;
                shotWaitTimer.Stop();
            }
            break;
        case STATE_MANUAL:
            if (intakePosition == true)
            { // intake goes down
                horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward);
            }
            else
            { // intake goes up
                horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse);
            };
            intakeMotorStageOne->Set(intakeSpeed); // set stage 1 and 2 to the manual speed
            intakeMotorStageTwo->Set(intakeSpeed);
            ballCount = -1;
            stageTwoOccupied = false;
            // YOU CAN NEVER LEAVE MANUAL!!!
            break;
        }
        shooterSensorPrevious = currentShooterSensorInput;
        stageOneSensorPrevious = currentSensorOneInput;
        stageTwoSensorPrevious = currentSensorTwoInput;
    } while (currentState != startState);
}
// connects to gamEpiece
void Intake::setIntakeDirection(IntakeDirection intakeDirection)
{
    // make sure you arent shooting when you try to tell intake what to do
    targetDirection = intakeDirection;
}

void Intake::setIntakePosition(bool position)
{
    intakePosition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed)
{
    intakeSpeed = (.7 * speed);
} // positive to intake, negative to outtake

// returns the ball count
int Intake::returnBallCount()
{
    return ballCount;
}
// adds a ball to the counter when the lower storage sensor is tripped

bool Intake::finishedShooting()
{
    return currentState == STATE_STOP;
}

bool Intake::checkSensor(frc::DigitalInput *sensor)
{
    bool sensorOutput = sensor->Get();
#ifdef TEST_BOARD
    return sensorOutput;
#else
    return !sensorOutput;
#endif
}

bool Intake::ballAtStageOne()
{
    return stageOneSensorPrevious;
}

bool Intake::ballAtStageTwo()
{
    return stageTwoOccupied;
}

void Intake::setBallCount(int theCount)
{   
    if(ballCount == 2 && theCount != 2){
        stageTwoOccupied = false;
    }
    if(theCount == 2){
        stageTwoOccupied = true;
    }
    ballCount = theCount;
}

void Intake::ballWasShot(){
    stageTwoOccupied = false;
    ballCount += -1;
    intakeMotorStageOne->Set(0);
    intakeMotorStageTwo->Set(0);
    // currentState = STATE_STOP;
    currentState = STATE_WAIT_AFTER_SHOT;
    shotWaitTimer.Reset();
    shotWaitTimer.Start();
    if(!checkSensor(&stageOneFlag) && ballCount == 1){
        ballCount = 0;
    }
}

void Intake::sendFeedback()
{ // debug
    std::string targetIntakeState = "";
    switch (targetDirection)
    {
    case (INTAKE):
        targetIntakeState = "Intake";
        break;
    case (OUTTAKE):
        targetIntakeState = "Outtake";
        break;
    case (NOTTAKE):
        targetIntakeState = "Nottake";
        break;
    case (MANUAL):
        targetIntakeState = "Manual";
        break;
    case (SHOOTING):
        targetIntakeState = "Shooting";
        break;
    }
    std::string currentstate = "";
    switch (currentState)
    {
    case (STATE_INTAKE_ONE_BALL):
        currentstate = "Intake one ball";
        break;
    case (STATE_INTAKE_TWO_BALL):
        currentstate = "Intake two ball";
        break;
    case (STATE_PRE_COUNT):
        currentstate = "Pre count";
        break;
    case (STATE_OUTTAKE):
        currentstate = "Outtake";
        break;
    case (STATE_STOP):
        currentstate = "Stop";
        break;
    case (STATE_MANUAL):
        currentstate = "Manual";
        break;
    case (STATE_SHOOT):
        currentstate = "Shooting";
        break;
    case STATE_INTAKE_BRING_BALL_IN:
        currentstate = "Bring ball in";
        break;
    case STATE_WAIT_AFTER_SHOT:
        currentstate = "After shot";
        break;
    }
    std::string intakePositionString = "";
    if (intakePosition)
    {
        intakePositionString = "intake is down";
    }
    else
    {
        intakePositionString = "intake is up";
    }
    bool stageOneSensor = checkSensor(&stageOneFlag);
    Feedback::sendString("Intake", "Current target direction", targetIntakeState.c_str());
    Feedback::sendBoolean("Intake", "State of Intake, true = up", intakePosition);
    Feedback::sendDouble("Intake", "Manual speed of intake/storage", intakeSpeed);
    Feedback::sendString("Intake", "current state of the intake", currentstate.c_str());
    Feedback::sendBoolean("Intake", "sensor stage one, true is tripped", stageOneSensor);
    Feedback::sendBoolean("Intake", "sensor stage two true is tripped", checkSensor(&stageTwoFlag));
    Feedback::sendBoolean("Intake", "sensor shooter true is tripped", checkSensor(&shooterFlag));
    Feedback::sendBoolean("Intake", "sensor stage half, true is tripped", checkSensor(&stageHalfFlag));
    Feedback::sendBoolean("Intake", "true is stage two is occupied", stageTwoOccupied);
    Feedback::sendBoolean("Intake", "does stage one need fixing", stageOneFixing);
    Feedback::sendDouble("thunderdashboard", "ballcount", ballCount);
    Feedback::sendString("Intake", "current manual position of the intake", intakePositionString.c_str());
    Feedback::sendDouble("Intake", "current manual speed of the intake stage one/two", intakeSpeed);
    Feedback::sendDouble("Intake", "stage 1 temperature (F)", intakeMotorStageOne->GetMotorTemperatureFarenheit());
    Feedback::sendDouble("Intake", "stage 2 temperature (F)", intakeMotorStageTwo->GetMotorTemperatureFarenheit());

    Feedback::sendDouble("thunderdashboard", "stage1", stageOneSensor);
    Feedback::sendDouble("thunderdashboard", "stage2", stageTwoOccupied);
    Feedback::sendDouble("Intake", "shot wait timer", shotWaitTimer.Get().value());
}
