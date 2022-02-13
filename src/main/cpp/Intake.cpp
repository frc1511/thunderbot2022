#include "Intake.h"
#include "GamEpiece.h"

const double kSpeedStageOne = .7;
const double kSpeedStageTwo = .7;
const double kSpeedStageTwoSlow = .3;
const double kReverseSpeedStageOne = -.7;
const double kReverseSpeedStageTwo = -.7;

Intake::Intake() : intakeMotorStageOne(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage1)),
                   intakeMotorStageTwo(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage2))
{
}

Intake::~Intake()
{
}

void Intake::resetToMode(MatchMode mode)
{
    horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse);
    intakeMotorStageOne->Set(0); // stage 1 stop
    intakeMotorStageTwo->Set(0);
    intakePosition = false;
    intakeSpeed = 0;
    ballCount = 0;        // set the ball count to zero
    stageOneSensorPrevious = stageOneFlag.Get(); 
    stageTwoSensorPrevious = stageTwoFlag.Get(); 
    shooterSensorPrevious = shooterFlag.Get();
    stageTwoOccupied = 0; // stage 2 isn't occupied 
    currentState = STATE_STOP;
    targetDirection = NOTTAKE;
    
}
void Intake::ballCountIntake(bool currentsensoroneinput, bool currentsensortwoinput)
{
    if (stageOneSensorPrevious == false && currentsensoroneinput == true)
    {
        ballCount = ballCount + 1;
    }

    if (stageTwoSensorPrevious == true && currentsensortwoinput == false)
    {
        stageTwoOccupied = true;
    }
    stageOneSensorPrevious = currentsensoroneinput;
    stageTwoSensorPrevious = currentsensortwoinput;
}
void Intake::ballCountOuttake(bool currentsensoroneinput, bool currentsensortwoinput)
{

    if (stageOneSensorPrevious == true && currentsensoroneinput == false)
    {
        ballCount = ballCount - 1;
    }

    if (stageTwoSensorPrevious == false && currentsensortwoinput == true)
    {
        stageTwoOccupied = false;
    }
    stageOneSensorPrevious = currentsensoroneinput;
    stageTwoSensorPrevious = currentsensortwoinput;
}
void Intake::switchStates() // command switches
{
    switch (targetDirection)
    {
    case INTAKE:
        if (ballCount == 2)
        {
            currentState = STATE_INTAKE2Ball;
        }
        else
        {
            currentState = STATE_INTAKE1Ball;
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
        };
        currentState = STATE_SHOOT;
        break;
    }
}
void Intake::process()
{

    bool currentsensoroneinput = shooterFlag.Get();
    bool currentsensortwoinput = stageOneFlag.Get();
    bool currentshootersensorinput = stageTwoFlag.Get();
    // the desired actions of the intake motors
    switch (currentState)
    { // extend piston to retract intake, retract piston to extend/deploy intake
    case STATE_INTAKE1Ball:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward); // deploy intake, forward is forward
        intakeMotorStageOne->Set(kSpeedStageOne);                   // stage 1 go
        intakeMotorStageTwo->Set(kSpeedStageTwoSlow);               // stage 2 go, slow
        switchStates();
        ballCountIntake(currentsensoroneinput, currentsensortwoinput);
        break;
    case STATE_INTAKE2Ball:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
        intakeMotorStageOne->Set(0);                                // stage 1 stop
        intakeMotorStageTwo->Set(0);                                // stage 2 stop
        switchStates();
        ballCountIntake(currentsensoroneinput, currentsensortwoinput);
        break;
    case STATE_OUTTAKE:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kOff); // leave intake as is
        intakeMotorStageOne->Set(kReverseSpeedStageOne);        // stage 1 go reverse
        intakeMotorStageTwo->Set(kReverseSpeedStageTwo);        // stage 2 go reverse
        switchStates();
        ballCountOuttake(currentsensoroneinput, currentsensortwoinput);
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
        switchStates();
        ballCountIntake(currentsensoroneinput, currentsensortwoinput);
        break;
    case STATE_SHOOT:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
        if (stageTwoOccupied == true)
        {
            intakeMotorStageTwo->Set(kSpeedStageTwo); // stage 2 go
            intakeMotorStageOne->Set(0);              // stage 1 stops
        }
        else
        {
            intakeMotorStageTwo->Set(0); // stage 2 stops
            intakeMotorStageOne->Set(0); // stage 1 stops
            currentState = STATE_STOP;   // go back to stop, which gets any other possible balls in position
        }

        if (currentshootersensorinput == false && shooterSensorPrevious == true)
        {
            ballCount = ballCount - 1;
            stageTwoOccupied = false;
        }

        shooterSensorPrevious = currentshootersensorinput;
        break;
    case STATE_MANUAL:
        if (intakePosition == true)
        { // intake goes down
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        { // intake goes up
            horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward);
        };
        intakeMotorStageOne->Set(intakeSpeed); // set stage 1 and 2 to the manual speed
        intakeMotorStageTwo->Set(intakeSpeed);
        ballCount = GamEpiece::BALL_COUNT_UNKNOWN;
        stageTwoOccupied = false;
        // YOU CAN NEVER LEAVE MANUAL!!!
        break;
    }
    shooterSensorPrevious = currentshootersensorinput;
    stageOneSensorPrevious = currentsensoroneinput;
    stageTwoSensorPrevious = currentsensortwoinput;
}
// connects to gamEpiece
void Intake::setIntakeDirection(IntakeDirection intakeDirection)
{
    // make sure you arent shooting when you try to tell intake what to do
    if (targetDirection != SHOOTING)
    {
        targetDirection = intakeDirection;
    }
}

void Intake::setIntakePosition(bool position)
{
    intakePosition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed)
{
    intakeSpeed = speed;
} // positive to intake, negative to outtake


// returns the ball count
int Intake::returnBallCount()
{
    return ballCount;
}
// adds a ball to the counter when the lower storage sensor is tripped

bool Intake::finishedShooting()
{
    return targetDirection == NOTTAKE;
}

void Intake::sendFeedback()
{ // debug
    std::string intakeState = "";
    switch (targetDirection)
    {
    case (INTAKE):
        intakeState = "Intake";
        break;
    case (OUTTAKE):
        intakeState = "Outtake";
        break;
    case (NOTTAKE):
        intakeState = "Nottake";
        break;
    case (MANUAL):
        intakeState = "Manual";
        break;
    case (SHOOTING):
        intakeState = "Shooting";
        break;
    }
    std::string currentstate = "";
    switch (currentState)
    {
    case (STATE_INTAKE1Ball):
        currentstate = "Intake 1 ball";
        break;
    case (STATE_INTAKE2Ball):
        currentstate = "Intake 2 ball";
        break;
    case (STATE_OUTTAKE):
        currentstate = "Outtake";
        break;
    case (STATE_STOP):
        currentstate = "Nottake";
        break;
    case (STATE_MANUAL):
        currentstate = "Manual";
        break;
    case (STATE_SHOOT):
        currentstate = "Shooting";
        break;
    }
    Feedback::sendString("Intake", "Current target direction", intakeState.c_str());
    Feedback::sendBoolean("Intake", "State of Intake, true = up", intakePosition);
    Feedback::sendDouble("Intake", "Manual speed of intake/storage", intakeSpeed);
    Feedback::sendString("Intake", "current state of the intake", currentstate.c_str());
    Feedback::sendBoolean("Intake", "sensor stage one, true is tripped", stageOneSensorPrevious);
    Feedback::sendBoolean("Intake", "sensor stage two true is tripped", stageTwoSensorPrevious);
    Feedback::sendBoolean("Intake", "sensor shooter true is tripped", shooterSensorPrevious);
    Feedback::sendBoolean("Intake", "true is stage two is occupied", stageTwoOccupied);
    Feedback::sendDouble("Intake", "ball count", ballCount); 


}
