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

// The tolerance of the hood position.
#define HOOD_TOLERANCE .005

// Speeds of the hood servo.
#define HOOD_SPEED_STOPPED 0
#define HOOD_SPEED_FORWARD .5
#define HOOD_SPEED_BACKWARD -.5

#define SHOOTER_TOLERANCE 10

// 1600 rpm 0.138 pot 1.53 m
// 1500 rpm 0.100 pot 0.70 m
// 1750 rpm 0.158 pot 2.265 m
// 1900 rpm 0.165 pot 2.83 m
// 2200 rpm 0.229 pot 3.877 m

#define NEW_INTERP_2_RPM 1500
#define NEW_INTERP_2_HOOD 0.0452 + HOOD_MIN_POS
#define NEW_INTERP_2_DIST 0.7_m

#define NEW_INTERP_1_RPM 1600
#define NEW_INTERP_1_HOOD 0.0832 + HOOD_MIN_POS
#define NEW_INTERP_1_DIST 1.53_m


//1.50 0.136

// #define NEW_INTERP_3_RPM 1600
// #define NEW_INTERP_3_

#define NEW_INTERP_3_RPM 1750
#define NEW_INTERP_3_HOOD 0.1032 + HOOD_MIN_POS
#define NEW_INTERP_3_DIST 2.265_m

#define NEW_INTERP_4_RPM 1900
#define NEW_INTERP_4_HOOD 0.1102 + HOOD_MIN_POS
#define NEW_INTERP_4_DIST 2.83_m

#define NEW_INTERP_5_RPM 2200
#define NEW_INTERP_5_HOOD 0.1742 + HOOD_MIN_POS
#define NEW_INTERP_5_DIST 3.877_m

// --- Preset values ---

// The hood position and shooter RPM when the robot is at the far wall.
#define WALL_HOOD_POS (HOOD_MIN_POS + .1531)
#define WALL_SHOOTER_RPM 2600
#define WALL_DISTANCE (4.02_m + 3_in)

// The hood position and shooter RPM when the robot is at the launch pad.
#define FAR_LAUNCH_PAD_HOOD_POS (HOOD_MIN_POS + .1311)
#define FAR_LAUNCH_PAD_SHOOTER_RPM 2200
// make button :D
// The hood position and shooter RPM when the robot is at the tarmac line.
#define NEAR_LAUNCH_PAD_HOOD_POS (HOOD_MIN_POS + .1253) // .680
#define NEAR_LAUNCH_PAD_SHOOTER_RPM 2000
#define NEAR_LAUNCH_PAD_DISTANCE 2.7117_m

// The hood position and shooter RPM when the robot is at the tarmac line.
#define TARMAC_LINE_HOOD_POS (HOOD_MIN_POS + .0703) // .625
#define TARMAC_LINE_SHOOTER_RPM 1800
#define TARMAC_LINE_DISTANCE 1.93_m

#define TARMAC_HOOD_POS (HOOD_MIN_POS + .0573) // 0.612   0.598   0.586
#define TARMAC_SHOOTER_RPM 1600
#define TARMAC_DISTANCE 1.242_m

// The hood position and shooter RPM when the robot is at the tarmac line.
#define HIGH_HUB_SHOT_HOOD_POS (HOOD_MIN_POS + .012)
#define HIGH_HUB_SHOT_SHOOTER_RPM 1600

// The hood position and shooter RPM when the robot is at the tarmac line.
#define LOW_HUB_SHOT_HOOD_POS (HOOD_MIN_POS + .111) // .641
#define LOW_HUB_SHOT_SHOOTER_RPM 1100

#define INTERPOLATION_1_HOOD_POS (HOOD_MIN_POS + .1093) // .664
#define INTERPOLATION_1_SHOOTER_RPM 1900
#define INTERPOLATION_1_DISTANCE 2.51_m

#define INTERPOLATION_2_HOOD_POS (HOOD_MIN_POS + .1453) // .700
#define INTERPOLATION_2_SHOOTER_RPM 2400
#define INTERPOLATION_2_DISTANCE 3.72_m
  
// #define HUB_LIMELIGHT_ANGLE 17.0777

Shooter::Shooter(Limelight* limelight)
  : limelight(limelight),
    shooterLeftMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::ShooterLeft)),
    shooterRightMotor(ThunderSparkMax::create(ThunderSparkMax::MotorID::ShooterRight)),
    shooterLeftPID(shooterLeftMotor->GetPIDController()),
    shooterRightPID(shooterRightMotor->GetPIDController())
#ifdef PETERS_INTERPOLATION
    ,hoodInterpolation({
        { NEW_INTERP_2_DIST, NEW_INTERP_2_HOOD },
        { NEW_INTERP_1_DIST, NEW_INTERP_1_HOOD },
        { NEW_INTERP_3_DIST, NEW_INTERP_3_HOOD },
        { NEW_INTERP_4_DIST, NEW_INTERP_4_HOOD },
        { NEW_INTERP_5_DIST, NEW_INTERP_5_HOOD },
        // { TARMAC_DISTANCE,           TARMAC_HOOD_POS          },
        // { TARMAC_LINE_DISTANCE,      TARMAC_LINE_HOOD_POS     },
        // { INTERPOLATION_1_DISTANCE,  INTERPOLATION_1_HOOD_POS },
        // { NEAR_LAUNCH_PAD_DISTANCE,  NEAR_LAUNCH_PAD_HOOD_POS },
        // { INTERPOLATION_2_DISTANCE,  INTERPOLATION_2_HOOD_POS },
        // { WALL_DISTANCE,             WALL_HOOD_POS            },
    }),
    rpmInterpolation({
        { NEW_INTERP_2_DIST, NEW_INTERP_2_RPM },
        { NEW_INTERP_1_DIST, NEW_INTERP_1_RPM },
        { NEW_INTERP_3_DIST, NEW_INTERP_3_RPM },
        { NEW_INTERP_4_DIST, NEW_INTERP_4_RPM },
        { NEW_INTERP_5_DIST, NEW_INTERP_5_RPM },
        // { TARMAC_DISTANCE,           TARMAC_SHOOTER_RPM          },
        // { TARMAC_LINE_DISTANCE,      TARMAC_LINE_SHOOTER_RPM     },
        // { INTERPOLATION_1_DISTANCE,  INTERPOLATION_1_SHOOTER_RPM },
        // { NEAR_LAUNCH_PAD_DISTANCE,  NEAR_LAUNCH_PAD_SHOOTER_RPM },
        // { INTERPOLATION_2_DISTANCE,  INTERPOLATION_2_SHOOTER_RPM },
        // { WALL_DISTANCE,             WALL_SHOOTER_RPM            },
    })
#endif
{
#ifndef PETERS_INTERPOLATION
    varsDistance = {HUB_LIMELIGHT_ANGLE, TARMAC_LINE_LIMELIGHT_ANGLE, NEAR_LAUNCH_PAD_LIMELIGHT_ANGLE, WALL_LIMELIGHT_ANGLE};
    varsHood = {HIGH_HUB_SHOT_HOOD_POS, TARMAC_LINE_HOOD_POS, NEAR_LAUNCH_PAD_HOOD_POS, WALL_HOOD_POS};
    varsSpeed = {HIGH_HUB_SHOT_SHOOTER_RPM, TARMAC_LINE_SHOOTER_RPM, NEAR_LAUNCH_PAD_SHOOTER_RPM, WALL_SHOOTER_RPM};
#endif
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
    shooterMode = LOW_HUB_SHOT;
}

void Shooter::process() {
    units::meter_t limeDist = limelight->getDistance();
    switch (shooterMode) {
        case ODOMETRY:
            // if (odometryMode == INTERPOLATION) {
#ifdef PETERS_INTERPOLATION
                if (limelight->hasTarget() && limeDist >= NEW_INTERP_2_DIST) {
                    targetHoodPosition = hoodInterpolation[limeDist].value();
                    targetRPM = rpmInterpolation[limeDist].value();
                }
#else
                
                //distance = limelight->getDistance();
                
                distance = limelight->getAngleVertical().value();
                for(unsigned int i = 0; i < varsDistance.size(); i++){
                    if(distance >= varsDistance[i]){
                        goodNumber = i;
                        break;
                    }
                }
                targetHoodPosition = interpolation(varsDistance[goodNumber], varsHood[goodNumber], varsDistance[goodNumber+1], varsHood[goodNumber+1], distance);
                targetRPM = interpolation(varsDistance[goodNumber], varsSpeed[goodNumber], varsDistance[goodNumber+1], varsSpeed[goodNumber+1], distance);
#endif
            // }
            // else if (odometryMode == CRAZY_MATH) {
                // if (limelight->hasTarget()) {
                    // ShotMath::Shot shot = shotMath.calculateShot(limelight->getDistance(), 0_mps, 0_deg);
                    // targetHoodPosition = shot.hoodPos;
                    // targetRPM = shot.shooterRPM;
                // }
            // }
            break;
        case FAR_LAUNCH_PAD:
            targetHoodPosition = FAR_LAUNCH_PAD_HOOD_POS;
            targetRPM = FAR_LAUNCH_PAD_SHOOTER_RPM;
            break;
        case NEAR_LAUNCH_PAD:
            targetHoodPosition = NEAR_LAUNCH_PAD_HOOD_POS;
            targetRPM = NEAR_LAUNCH_PAD_SHOOTER_RPM;
            break;
        case TARMAC_LINE:
            targetHoodPosition = TARMAC_LINE_HOOD_POS;
            targetRPM = TARMAC_LINE_SHOOTER_RPM;
            break;
        case TARMAC:
            targetHoodPosition = TARMAC_HOOD_POS;
            targetRPM = TARMAC_SHOOTER_RPM;
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
            if(hoodSpeedManual == 0 && hoodSpeedManualLast != 0){
                targetHoodPosition = readPotentiometer();
            }
            break;
    }
    
    if (!wantToShoot) {
        targetRPM = 0;
    }
    else if (!targetRPM) {
        targetRPM = NEAR_LAUNCH_PAD_SHOOTER_RPM;
    }
    
    if(reverse){
        shooterLeftMotor->Set(-.2);
        shooterRightMotor->Set(-.2);
    }
    else{
        shooterLeftPID->SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);
        shooterRightPID->SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);
    }

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
    // Below the target position. hi peter :D
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
    hoodSpeedManualLast = hoodSpeedManual;

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
    if(targetRPM <= 2400){
    return (shooterLeftMotor->GetVelocity() > targetRPM - SHOOTER_TOLERANCE) && // if left is speedy enough :D
           (shooterRightMotor->GetVelocity() > targetRPM - SHOOTER_TOLERANCE) && // if right is speedy enough :D
           (shooterLeftMotor->GetVelocity() < targetRPM +  SHOOTER_TOLERANCE) && // if left is speedy enough :D
           (shooterRightMotor->GetVelocity() < targetRPM + SHOOTER_TOLERANCE) && // if right is speedy enough :D
           ((fabs(readPotentiometer() - targetHoodPosition) < HOOD_TOLERANCE) || shooterMode == MANUAL) &&
           (targetRPM != 0); // if hood is in the right place
    }
    else{
        return (shooterLeftMotor->GetVelocity() > targetRPM - 25) && // if left is speedy enough :D
           (shooterRightMotor->GetVelocity() > targetRPM - 25) && // if right is speedy enough :D
           (shooterLeftMotor->GetVelocity() < targetRPM +  25) && // if left is speedy enough :D
           (shooterRightMotor->GetVelocity() < targetRPM + 25) && // if right is speedy enough :D
           ((fabs(readPotentiometer() - targetHoodPosition) < HOOD_TOLERANCE) || shooterMode == MANUAL) &&
           (targetRPM != 0); // if hood is in the right place
    }
}

bool Shooter::isShooterKindOfReady() {
    return (shooterLeftMotor->GetVelocity() > targetRPM - 75) && // if left is speedy enough :D
        (shooterRightMotor->GetVelocity() > targetRPM - 75) && // if right is speedy enough :D
        (shooterLeftMotor->GetVelocity() < targetRPM +  75) && // if left is speedy enough :D
        (shooterRightMotor->GetVelocity() < targetRPM + 75) && // if right is speedy enough :D
        ((fabs(readPotentiometer() - targetHoodPosition) < .02) || shooterMode == MANUAL) &&
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

#ifndef PETERS_INTERPOLATION

double Shooter::interpolation(double firstX, double firstY, double lastX,  double lastY, double distance){
    return (((lastY-firstY)/(lastX-firstX))*distance)+firstY-(firstX*((lastY-firstY)/(lastX-firstX)));
}
#endif


double Shooter::readPotentiometer(){
    return hoodPotentiometer.Get();
}

void Shooter::spinInReverse(bool reverseOrNot){
    /*if(reverseOrNot){
        shooterLeftPID->SetOutputRange(-SHOOTER_MAX_RPM, SHOOTER_MAX_RPM);
        shooterRightPID->SetOutputRange(-SHOOTER_MAX_RPM, SHOOTER_MAX_RPM);
    }
    else{
        shooterLeftPID->SetOutputRange(0,SHOOTER_MAX_RPM);
        shooterRightPID->SetOutputRange(0,SHOOTER_MAX_RPM);
    }*/
    reverse = reverseOrNot;
}

void Shooter::setOdometryMode(OdometryMode mode) {
    odometryMode = mode;
}

void Shooter::recordShooterValues(){
    std::ofstream MyFile;
    MyFile.open("/home/lvuser/shooterValues.txt", std::fstream::app);
    MyFile << "rpm: " << std::to_string(manualRPM) << ", pot value: " << std::to_string(readPotentiometer()) << ", limelight angle: " << std::to_string(limelight->getAngleVertical().value()) << ", limelight distance: " << std::to_string(limelight->getDistance().value()) << "\n";
    MyFile.close();
}

void Shooter::changeShooterPid(bool goUp){
    if(goUp){
        shooterP += .0001;
    }
    else{
        shooterP += -.0001;
    }
    shooterLeftPID->SetP(shooterP, 0);
    shooterRightPID->SetP(shooterP, 0);
}

void Shooter::sendFeedback() {
    std::string modeString = "";
    switch (shooterMode) {
        case ODOMETRY:
            modeString = "Odometry";
            break;
        case FAR_LAUNCH_PAD:
            modeString = "Far Launch Pad";
            break;
        case NEAR_LAUNCH_PAD:
            modeString = "Near Launch Pad";
            break;
        case TARMAC_LINE:
            modeString = "Tarmac Line";
            break;
        case TARMAC:
            modeString = "Tarmac";
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
    Feedback::sendDouble("shooter", "shooter p value for pid", shooterP);

    Feedback::sendDouble("thunderdashboard", "shooter_hood", (readPotentiometer() * 100));
}