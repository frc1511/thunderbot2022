#include "Shooter.h"
#include "rev/RelativeEncoder.h"

#define SHOOTER_MAX_VOLTAGE 12
#define SHOOTER_MAX_AMPS 40

// PID values of the shooter wheels.
#define SHOOTER_P_VALUE .0015
#define SHOOTER_I_VALUE 0
#define SHOOTER_D_VALUE 0
#define SHOOTER_I_ZONE_VALUE 0
#define SHOOTER_FF_VALUE .000187

// Minimum / maximum hood servo positions.
#define HOOD_MIN_POS 0.531 //0.433
#define HOOD_MAX_POS (HOOD_MIN_POS + .154) //0.7

// The maximum RPM of the shooter wheels.
#define SHOOTER_MAX_RPM 2700 // 5700

// The tolerance of the hood position.
#define HOOD_TOLERANCE .005

// Speeds of the hood servo.
#define HOOD_SPEED_STOPPED 0
#define HOOD_SPEED_FORWARD .5
#define HOOD_SPEED_BACKWARD -.5

#define SHOOTER_TOLERANCE 40

// --- Preset values ---

// The hood position and shooter RPM when the robot is right next to the hub.
#define HUB_HOOD_POS (HOOD_MIN_POS + .0187)
#define HUB_SHOOTER_RPM 2000

// The hood position and shooter RPM when the robot is at the far wall.
#define WALL_HOOD_POS (HOOD_MIN_POS + .1531)
#define WALL_SHOOTER_RPM 2600

// The hood position and shooter RPM when the robot is at the launch pad.
#define LAUNCH_PAD_HOOD_POS (HOOD_MIN_POS + .1311)
#define LAUNCH_PAD_SHOOTER_RPM 2200

// The hood position and shooter RPM when the robot is at the tarmac line.
#define TARMAC_LINE_HOOD_POS (HOOD_MIN_POS + .1038)
#define TARMAC_LINE_SHOOTER_RPM 2000

// The hood position and shooter RPM when the robot is at the tarmac line.
#define HIGH_HUB_SHOT_HOOD_POS (HOOD_MIN_POS + .019) //.550
#define HIGH_HUB_SHOT_SHOOTER_RPM 1600

// The hood position and shooter RPM when the robot is at the tarmac line.
#define LOW_HUB_SHOT_HOOD_POS (HOOD_MIN_POS + .111) // .641
#define LOW_HUB_SHOT_SHOOTER_RPM 1100

Shooter::Shooter(Limelight* limelight)
  : limelight(limelight),
    shooterLeftMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::ShooterLeft)),
    shooterRightMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::ShooterRight)),
    shooterLeftPID(shooterLeftMotor->GetPIDController()),
    shooterRightPID(shooterRightMotor->GetPIDController())
{
    configureMotors();
}

Shooter::~Shooter() {

}

void Shooter::configureMotors() {
    // --- Shooting motor config ---
    shooterLeftMotor->RestoreFactoryDefaults();
    shooterRightMotor->RestoreFactoryDefaults();
    // Make the motors coast.
    shooterLeftMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
    shooterRightMotor->SetIdleMode(ThunderSparkMax::IdleMode::COAST);
    // Make sure the motors don't draw more than 12V.
    shooterLeftMotor->EnableVoltageCompensation(SHOOTER_MAX_VOLTAGE);
    shooterRightMotor->EnableVoltageCompensation(SHOOTER_MAX_VOLTAGE);
    // Make sure the motors don't draw more than 40A.
    shooterLeftMotor->SetSmartCurrentLimit(SHOOTER_MAX_AMPS);
    shooterRightMotor->SetSmartCurrentLimit(SHOOTER_MAX_AMPS);
    // One is inverted.
    shooterLeftMotor->SetInverted(false);
    shooterRightMotor->SetInverted(true);

    
    // PID values.
    shooterLeftPID->SetP(SHOOTER_P_VALUE, 0);
    shooterRightPID->SetP(SHOOTER_P_VALUE, 0);
    shooterLeftPID->SetI(SHOOTER_I_VALUE, 0);
    shooterRightPID->SetI(SHOOTER_I_VALUE, 0);
    shooterLeftPID->SetD(SHOOTER_D_VALUE, 0);
    shooterRightPID->SetD(SHOOTER_D_VALUE, 0);
    shooterLeftPID->SetIZone(SHOOTER_I_ZONE_VALUE, 0);
    shooterRightPID->SetIZone(SHOOTER_I_ZONE_VALUE, 0);
    shooterLeftPID->SetFF(SHOOTER_FF_VALUE, 0);
    shooterRightPID->SetFF(SHOOTER_FF_VALUE, 0);
    shooterLeftPID->SetOutputRange(0,SHOOTER_MAX_RPM);
    shooterRightPID->SetOutputRange(0,SHOOTER_MAX_RPM);
}

void Shooter::doPersistentConfiguration() {
    configureMotors();
    shooterRightMotor->BurnFlash();
    shooterLeftMotor->BurnFlash();
}

void Shooter::resetToMode(MatchMode mode) {
    shooterLeftMotor->Set(0);
    shooterRightMotor->Set(0);
    hoodServo.Set(.5);
    hoodSpeedManual = 0;
    wantToShoot = false;
    shooterMode = TARMAC_LINE;
}



void Shooter::process() {
    switch (shooterMode) {
        case ODOMETRY:
            //targetHoodPosition = hoodInterpolation[limelight->getAngleVertical()].value();
            //targetRPM = rpmInterpolation[limelight->getAngleVertical()].value();
            distance = limelight->getDistance();
            for(unsigned int i = 0; i <= xVarsHood.size(); i++){
                if(distance >= xVarsHood[i]){
                    goodNumber = i;
                    break;
                }
            }
            targetHoodPosition = interpolation(xVarsHood[goodNumber], yVarsHood[goodNumber], xVarsHood[goodNumber+1], yVarsHood[goodNumber+1], distance);
            targetRPM = interpolation(xVarsSpeed[goodNumber], yVarsSpeed[goodNumber], xVarsSpeed[goodNumber+1], yVarsSpeed[goodNumber+1], distance);
            break;
        case LAUNCH_PAD:
            targetHoodPosition = LAUNCH_PAD_HOOD_POS;
            targetRPM = LAUNCH_PAD_SHOOTER_RPM;
            break;
        case TARMAC_LINE:
            targetHoodPosition = TARMAC_LINE_HOOD_POS;
            targetRPM = TARMAC_LINE_SHOOTER_RPM;
            break;
        case HIGH_HUB_SHOT:
            targetHoodPosition = HIGH_HUB_SHOT_HOOD_POS;
            targetRPM = HIGH_HUB_SHOT_SHOOTER_RPM;
            break;
        case LOW_HUB_SHOT:
            targetHoodPosition = LOW_HUB_SHOT_HOOD_POS;
            targetRPM = LOW_HUB_SHOT_SHOOTER_RPM;
            break;
        case MANUAL:
            targetRPM = manualRPM;
            break;
    }
    
    if (!wantToShoot) {
        targetRPM = 0;
    }
    
    if(reverse){
        targetRPM = -500;
    }
    shooterLeftPID->SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);
    shooterRightPID->SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);
    

    // gets the position of the hood potentiometer
    double hoodPosition = readPotentiometer();
    // 🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲
    double servoSpeed = HOOD_SPEED_STOPPED;

    // If servo is given a speed to go to
    if (shooterMode == MANUAL) {
        servoSpeed = hoodSpeedManual;
    }
    // If hood position is within the tolerance level to the target position.
    else if (fabs(hoodPosition - targetHoodPosition) <= HOOD_TOLERANCE) {
        servoSpeed = HOOD_SPEED_STOPPED;
    }
    // Below the target position.
    else if (hoodPosition < targetHoodPosition) {
        servoSpeed = HOOD_SPEED_FORWARD;
    }
    // Above the target position.
    else if (hoodPosition > targetHoodPosition) {
        servoSpeed = HOOD_SPEED_BACKWARD;
    }

    // Stop the servo if it cannot go forward anymore.
    if (hoodPosition >= HOOD_MAX_POS && servoSpeed > HOOD_SPEED_STOPPED) {
        servoSpeed = HOOD_SPEED_STOPPED;
    }
    // Stop the servo if it cannot go backwards anymore.
    else if (hoodPosition <= HOOD_MIN_POS && servoSpeed < HOOD_SPEED_STOPPED) {
        servoSpeed = HOOD_SPEED_STOPPED;
    }

    // Set the speed of the servo.
    if(fabs(servoSpeed) >= .05){
        if(servoSpeed > 0){
            hoodServo.Set(0); //might need to reverse
        }
        else{
            hoodServo.Set(1);
        }
    }
    else{
        hoodServo.Set(.5);
    }
}

void Shooter::setShooterSpinup(bool shouldShoot) {
    wantToShoot = shouldShoot;
}
// sees if the shooter is ready
bool Shooter::isShooterReady() {
    return (shooterLeftMotor->GetVelocity() > targetRPM - SHOOTER_TOLERANCE) && // if left is speedy enough :D
           (shooterRightMotor->GetVelocity() > targetRPM - SHOOTER_TOLERANCE) && // if right is speedy enough :D
           ((fabs(readPotentiometer() - targetHoodPosition) < HOOD_TOLERANCE) || shooterMode == MANUAL) &&
           (targetRPM != 0); // if hood is in the right place
}
// allows manual control of the hood speed
void Shooter::setHoodManual(double speed) {
    hoodSpeedManual = speed;
    if(speed){
        shooterMode = MANUAL;
    }

}

void Shooter::setShooterMode(ShooterMode targetMode) {
    shooterMode = targetMode;
}

void Shooter::changeManualSpeed(bool increaseOrDecrease){
    if(increaseOrDecrease && manualRPM < SHOOTER_MAX_RPM){ // dont let it go faster than intended
        manualRPM += 100;
    }
    else if(increaseOrDecrease == false && manualRPM > 0){ // dont let it shoot backwards cause thats bad
        manualRPM -= 100;
    }
    shooterMode = MANUAL;
}

double Shooter::interpolation(double firstX, double firstY, double lastX,  double lastY, double distance){
    return (((lastY-firstY)/(lastX-firstX))*distance)+firstY-(firstX*((lastY-firstY)/(lastX-firstX)));
}

double Shooter::readPotentiometer(){
    return hoodPotentiometer.Get();
}

void Shooter::spinInReverse(bool reverseOrNot){
    if(reverseOrNot){
        shooterLeftPID->SetOutputRange(-SHOOTER_MAX_RPM, SHOOTER_MAX_RPM);
        shooterRightPID->SetOutputRange(-SHOOTER_MAX_RPM, SHOOTER_MAX_RPM);
    }
    else{
        shooterLeftPID->SetOutputRange(0,SHOOTER_MAX_RPM);
        shooterRightPID->SetOutputRange(0,SHOOTER_MAX_RPM);
    }
    reverse = reverseOrNot;
}

void Shooter::sendFeedback() {
    std::string modeString = "";
    switch (shooterMode) {
        case ODOMETRY:
            modeString = "Odometry";
            break;
        case LAUNCH_PAD:
            modeString = "Launch Pad";
            break;
        case TARMAC_LINE:
            modeString = "Tarmac Line";
            break;
        case HIGH_HUB_SHOT:
            modeString = "High Hub Shot";
            break;
        case LOW_HUB_SHOT:
            modeString = "Low Hub Shot";
            break;
        case MANUAL:
            modeString = "Manual";
            break;
    }

    Feedback::sendString("shooter", "Mode", modeString.c_str());
    Feedback::sendDouble("shooter", "Left velocity (RPM)", shooterLeftMotor->GetVelocity());
    Feedback::sendDouble("shooter", "Right velocity (RPM)", shooterRightMotor->GetVelocity());
    Feedback::sendDouble("shooter", "Hood position", readPotentiometer());

    Feedback::sendBoolean("shooter", "A. Want to shoot", wantToShoot);
    Feedback::sendDouble("shooter", "Hood speed manual", hoodSpeedManual);
    Feedback::sendDouble("shooter", "Target RPM", targetRPM);
    Feedback::sendDouble("shooter", "Target hood position", targetHoodPosition);
    Feedback::sendDouble("shotoer", "manual RPM", manualRPM);
    Feedback::sendDouble("shooter", "manual hood speed", hoodSpeedManual);
    Feedback::sendDouble("shooter", "left temperature (F)", shooterLeftMotor->GetMotorTemperatureFarenheit());
    Feedback::sendDouble("shooter", "right temperature (F)", shooterRightMotor->GetMotorTemperatureFarenheit());
    Feedback::sendBoolean("shooter", "ready to shoot", isShooterReady());

    Feedback::sendDouble("thunderdashboard", "shooter_hood", (readPotentiometer() * 100));
}