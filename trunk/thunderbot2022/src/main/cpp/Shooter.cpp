#include "Shooter.h"
#include "rev/RelativeEncoder.h"

#define SHOOTER_MAX_VOLTAGE 12
#define SHOOTER_MAX_AMPS 40

// PID values of the shooter wheels.
#define SHOOTER_P_VALUE .00008
#define SHOOTER_I_VALUE 0
#define SHOOTER_D_VALUE 0
#define SHOOTER_I_ZONE_VALUE 0
#define SHOOTER_FF_VALUE .000187

// Minimum / maximum hood servo positions.
#define HOOD_MIN_POS .48
#define HOOD_MAX_POS (HOOD_MIN_POS + .486)

// The maximum RPM of the shooter wheels.
#define SHOOTER_MAX_RPM 5700

// The tolerance of the hood position.
#define HOOD_TOLERANCE .01

// Speeds of the hood servo.
#define HOOD_SPEED_STOPPED 0
#define HOOD_SPEED_FORWARD .5
#define HOOD_SPEED_BACKWARD -.5

#define SHOOTER_TOLERANCE 100

// --- Preset values ---

// The hood position and shooter RPM when the robot is right next to the hub.
#define HUB_HOOD_POS 0
#define HUB_SHOOTER_RPM 4000

// The hood position and shooter RPM when the robot is at the far wall.
#define WALL_HOOD_POS 0
#define WALL_SHOOTER_RPM 4000

// The hood position and shooter RPM when the robot is at the launch pad.
#define LAUNCH_PAD_HOOD_POS 0
#define LAUNCH_PAD_SHOOTER_RPM 3500

// The hood position and shooter RPM when the robot is at the tarmac line.
#define TARMAC_LINE_HOOD_POS 0
#define TARMAC_LINE_SHOOTER_RPM 3250

Shooter::Shooter(Limelight* limelight)
  : limelight(limelight),
    shooterLeftEncoder(shooterLeftMotor.GetEncoder()),
    shooterRightEncoder(shooterRightMotor.GetEncoder()),
    shooterLeftPID(shooterLeftMotor.GetPIDController()),
    shooterRightPID(shooterRightMotor.GetPIDController()) {

    // --- Shooting motor config ---

    shooterLeftMotor.RestoreFactoryDefaults();
    shooterRightMotor.RestoreFactoryDefaults();
    // Make the motors coast.
    shooterLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    shooterRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // Make sure the motors don't draw more than 12V.
    shooterLeftMotor.EnableVoltageCompensation(SHOOTER_MAX_VOLTAGE);
    shooterRightMotor.EnableVoltageCompensation(SHOOTER_MAX_VOLTAGE);
    // Make sure the motors don't draw more than 40A.
    shooterLeftMotor.SetSmartCurrentLimit(SHOOTER_MAX_AMPS);
    shooterRightMotor.SetSmartCurrentLimit(SHOOTER_MAX_AMPS);
    // Inverted?
    shooterLeftMotor.SetInverted(false);
    shooterRightMotor.SetInverted(false);
    // PID values.
    shooterLeftPID.SetP(SHOOTER_P_VALUE, 0);
    shooterRightPID.SetP(SHOOTER_P_VALUE, 0);
    shooterLeftPID.SetI(SHOOTER_I_VALUE, 0);
    shooterRightPID.SetI(SHOOTER_I_VALUE, 0);
    shooterLeftPID.SetD(SHOOTER_D_VALUE, 0);
    shooterRightPID.SetD(SHOOTER_D_VALUE, 0);
    shooterLeftPID.SetIZone(SHOOTER_I_ZONE_VALUE, 0);
    shooterRightPID.SetIZone(SHOOTER_I_ZONE_VALUE, 0);
    shooterLeftPID.SetFF(SHOOTER_FF_VALUE, 0);
    shooterRightPID.SetFF(SHOOTER_FF_VALUE, 0);
}

Shooter::~Shooter() {

}

void Shooter::resetToMode(MatchMode mode) {
    shooterLeftMotor.Set(0);
    shooterRightMotor.Set(0);
}

void Shooter::sendFeedback() {
    std::string modeString = "";
    switch (mode) {
        case ODOMETRY:
            modeString = "Odometry";
            break;
        case LAUNCH_PAD:
            modeString = "Launch Pad";
            break;
        case TARMAC_LINE:
            modeString = "Tarmac Line";
            break;
        case MANUAL:
            modeString = "Manual";
            break;
    }

    Feedback::sendString("shooter", "Mode", modeString.c_str());
    Feedback::sendDouble("shooter", "Left velocity (RPM)", shooterLeftEncoder.GetVelocity());
    Feedback::sendDouble("shooter", "Right velocity (RPM)", shooterRightEncoder.GetVelocity());
    Feedback::sendDouble("shooter", "Hood position", hoodPotentiometer.Get());

    Feedback::sendBoolean("shooter", "Want to shoot", wantToShoot);
    Feedback::sendDouble("shooter", "Hood speed manual", hoodSpeedManual);
    Feedback::sendDouble("shooter", "Target RPM", targetRPM);
    Feedback::sendDouble("shooter", "Target hood position", targetHoodPosition);
}

void Shooter::process() {
    switch (mode) {
        case ODOMETRY:
            // TODO Align the hood to high hub and set shooter speed using
            // either limelight or the position of the robot on the field.
            break;
        case LAUNCH_PAD:
            targetHoodPosition = LAUNCH_PAD_HOOD_POS;
            targetRPM = LAUNCH_PAD_SHOOTER_RPM;
            break;
        case TARMAC_LINE:
            targetHoodPosition = TARMAC_LINE_HOOD_POS;
            targetRPM = TARMAC_LINE_SHOOTER_RPM;
            break;
        case MANUAL:
            targetRPM = LAUNCH_PAD_SHOOTER_RPM;
            break;
    }
    
    if (!wantToShoot) {
        targetRPM = 0;
    }
    
    shooterLeftPID.SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);
    shooterRightPID.SetReference(targetRPM, rev::CANSparkMax::ControlType::kVelocity);

    // gets the position of the hood potentiometer
    double hoodPosition = hoodPotentiometer.Get();
    // 🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲🗲
    double servoSpeed = HOOD_SPEED_STOPPED;

    // If servo is given a speed to go to
    if (hoodSpeedManual != 0) {
        servoSpeed = hoodSpeedManual;
    }
    // If hood position is within the tolerance level to the target position.
    else if (abs(hoodPosition - targetHoodPosition) <= HOOD_TOLERANCE) {
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
    hoodServo.SetSpeed(servoSpeed);
}

void Shooter::setShooterSpinup(bool shouldShoot) {
    wantToShoot = shouldShoot;
}
// sees if the shooter is ready
bool Shooter::isShooterReady() {
    return (shooterLeftEncoder.GetVelocity() > targetRPM - SHOOTER_TOLERANCE) &&
           (shooterRightEncoder.GetVelocity() > targetRPM - SHOOTER_TOLERANCE) &&
           (abs(hoodPotentiometer.Get() - targetHoodPosition) < HOOD_TOLERANCE);
}
// allows manual control of the hood speed
void Shooter::setHoodManual(double speed) {
    hoodSpeedManual = speed;

}

void Shooter::setShooterMode(ShooterMode targetMode) {
    mode = targetMode;
}
