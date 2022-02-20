#include "Intake.h"

#define INTAKE_MAX_VOLTAGE 12
#define INTAKE_MAX_AMPERAGE 30

const double kSpeedStageOne = .4; // used for intaking
const double kSpeedStageTwo = .3; // used for shooting
const double kSpeedStageTwoSlow = .3; // used for intaking into stage two
const double kReverseSpeedStageOne = -.4; // used for outtaking
const double kReverseSpeedStageTwo = -.3; // used for outtaking

Intake::Intake() : intakeMotorStageOne(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage1)),
                   intakeMotorStageTwo(ThunderSparkMax::create(ThunderSparkMax::MotorID::StorageStage2)){
    configureMotors();
}

Intake::~Intake(){
}

void Intake::configureMotors(){
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

void Intake::doPersistentConfiguration(){
    configureMotors();
    intakeMotorStageOne->BurnFlash();
    intakeMotorStageTwo->BurnFlash();
}

void Intake::resetToMode(MatchMode mode){
    if (mode != MatchMode::MODE_DISABLED) {
        intakeMotorStageOne->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
        intakeMotorStageTwo->SetIdleMode(ThunderSparkMax::IdleMode::BRAKE);
    } else {
        intakeMotorStageOne->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
        intakeMotorStageTwo->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
    }
    horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse);
    intakeMotorStageOne->Set(0); // stage 1 stop
    intakeMotorStageTwo->Set(0);
    intakePosition = false;
    intakeSpeed = 0;
    ballCount = 0;        // set the ball count to zero
    stageOneSensorPrevious = checkSensor(&stageOneFlag); 
    stageTwoSensorPrevious = checkSensor(&stageTwoFlag); 
    shooterSensorPrevious = checkSensor(&shooterFlag);
    stageTwoOccupied = 0; // stage 2 isn't occupied 
    currentState = STATE_STOP;
    targetDirection = NOTTAKE;
    
}
void Intake::ballCountIntake(bool currentSensorOneInput, bool currentSensorTwoInput){
    if (stageOneSensorPrevious == false && currentSensorOneInput == true){
        ballCount ++;
    }

    if (stageTwoSensorPrevious == true && currentSensorTwoInput == false){
        stageTwoOccupied = true;
    }
    stageOneSensorPrevious = currentSensorOneInput;
    stageTwoSensorPrevious = currentSensorTwoInput;
}
void Intake::ballCountOuttake(bool currentSensorOneInput, bool currentSensorTwoInput){

    if (stageOneSensorPrevious == true && currentSensorOneInput == false){
        ballCount --;
    }

    if (stageTwoSensorPrevious == false && currentSensorTwoInput == true){
        stageTwoOccupied = false;
    }
    stageOneSensorPrevious = currentSensorOneInput;
    stageTwoSensorPrevious = currentSensorTwoInput;
}
void Intake::switchStates() // command switches
{
    switch (targetDirection){
    case INTAKE:
        if (ballCount == 2)
        {
            currentState = STATE_INTAKE_TWO_BALL;
        }
        else
        {
            currentState = STATE_INTAKE_ONE_BALL;
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
        else{
            currentState = STATE_SHOOT;
        }
        break;
    }
}
void Intake::process(){

    bool currentSensorOneInput = checkSensor(&stageOneFlag);
    bool currentSensorTwoInput = checkSensor(&stageTwoFlag);
    bool currentShooterSensorInput = checkSensor(&shooterFlag);
    // the desired actions of the intake motors
    switch (currentState){ // extend piston to retract intake, retract piston to extend/deploy intake
    case STATE_INTAKE_ONE_BALL:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kForward); // deploy intake, forward is forward
        intakeMotorStageOne->Set(kSpeedStageOne);                   // stage 1 go
        if(stageTwoOccupied) { //ball in stage two
            intakeMotorStageTwo->Set(0);                            // stage 2 go, slow
        }
        else{
            intakeMotorStageTwo->Set(kSpeedStageTwoSlow);
        }
        switchStates();
        ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
        break;
    case STATE_INTAKE_TWO_BALL:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kReverse); // retract intake
        intakeMotorStageOne->Set(0);                                // stage 1 stop
        intakeMotorStageTwo->Set(0);                                // stage 2 stop
        switchStates();
        ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
        break;
    case STATE_OUTTAKE:
        horizontalIntake.Set(frc::DoubleSolenoid::Value::kOff); // leave intake as is
        intakeMotorStageOne->Set(kReverseSpeedStageOne);        // stage 1 go reverse
        intakeMotorStageTwo->Set(kReverseSpeedStageTwo);        // stage 2 go reverse
        switchStates();
        ballCountOuttake(currentSensorOneInput, currentSensorTwoInput);
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
        ballCountIntake(currentSensorOneInput, currentSensorTwoInput);
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

        if (currentShooterSensorInput == false && shooterSensorPrevious == true)
        {
            ballCount = ballCount - 1;
            stageTwoOccupied = false;
        }

        shooterSensorPrevious = currentShooterSensorInput;
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
}
// connects to gamEpiece
void Intake::setIntakeDirection(IntakeDirection intakeDirection){
    // make sure you arent shooting when you try to tell intake what to do
    targetDirection = intakeDirection;
}

void Intake::setIntakePosition(bool position){
    intakePosition = position;
} // true is down, false is up

void Intake::setIntakeSpeed(double speed){
    intakeSpeed = (.7*speed);
} // positive to intake, negative to outtake


// returns the ball count
int Intake::returnBallCount(){
    return ballCount;
}
// adds a ball to the counter when the lower storage sensor is tripped

bool Intake::finishedShooting(){
    return currentState == STATE_STOP;
}

bool Intake::checkSensor(frc::DigitalInput* sensor){
    bool sensorOutput = sensor->Get();
    #ifdef TEST_BOARD
        return sensorOutput;
    #else
        return !sensorOutput;
    #endif
        
}

bool Intake::ballAtStageOne(){
    return stageOneSensorPrevious;
}

void Intake::sendFeedback(){ // debug
    std::string targetIntakeState = "";
    switch (targetDirection){
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
    switch (currentState){
    case (STATE_INTAKE_ONE_BALL):
        currentstate = "Intake one ball";
        break;
    case (STATE_INTAKE_TWO_BALL):
        currentstate = "Intake two ball";
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
    }
    std::string intakePositionString = "";
    if(intakePosition){
        intakePositionString = "intake is down";
    }
    else{
        intakePositionString = "intake is up";
    }

    Feedback::sendString("Intake", "Current target direction", targetIntakeState.c_str());
    Feedback::sendBoolean("Intake", "State of Intake, true = up", intakePosition);
    Feedback::sendDouble("Intake", "Manual speed of intake/storage", intakeSpeed);
    Feedback::sendString("Intake", "current state of the intake", currentstate.c_str());
    Feedback::sendBoolean("Intake", "sensor stage one, true is tripped", checkSensor(&stageOneFlag));
    Feedback::sendBoolean("Intake", "sensor stage two true is tripped", checkSensor(&stageTwoFlag));
    Feedback::sendBoolean("Intake", "sensor shooter true is tripped", checkSensor(&shooterFlag));
    Feedback::sendBoolean("Intake", "true is stage two is occupied", stageTwoOccupied);
    Feedback::sendDouble("Intake", "ball count", ballCount); 
    Feedback::sendString("Intake", "current manual position of the intake", intakePositionString.c_str());
    Feedback::sendDouble("Intake", "current manual speed of the intake stage one/two", intakeSpeed);
    Feedback::sendDouble("Intake", "stage 1 temperature (F)", intakeMotorStageOne->GetMotorTemperatureFarenheit());
    Feedback::sendDouble("Intake", "stage 2 temperature (F)", intakeMotorStageTwo->GetMotorTemperatureFarenheit());

    Feedback::sendDouble("thunderdashboard", "stage1", stageOneSensorPrevious);
    Feedback::sendDouble("thunderdashboard", "stage2", stageTwoOccupied );
}
