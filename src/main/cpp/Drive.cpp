#include "Drive.h"

// The path to the file to save encoder offsets into.
#define ENCODER_OFFSETS_FILE_NAME "/home/lvuser/magnetic_encoder_offsets.txt"

// The circumference of the drive wheels.
#define DRIVE_WHEEL_CIRCUMFERENCE 0.21 // meters

/**
 * Drive encoder value after 1 foot.
 */
#define DRIVE_FOOT_TO_ENDODER_FACTOR 6.51

/**
 * The gear ratio of the drive motor (5.25:1).
 */
// #define DRIVE_GEAR_RATIO 5.25

/**
 * The coefficient used to convert rotations of the NEO motors into a distance
 * traveled in meters.
 */
#define DRIVE_ENCODER_TO_METER_FACTOR (1 / (DRIVE_FOOT_TO_ENDODER_FACTOR * 3.28084)) // (DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO)

/**
 * The coefficient used to convert a distance in meters into a number of
 * rotations of the NEO motors.
 */
#define DRIVE_METER_TO_ENCODER_FACTOR (DRIVE_FOOT_TO_ENDODER_FACTOR * 3.28084) // (DRIVE_GEAR_RATIO / DRIVE_WHEEL_CIRCUMFERENCE)

/**
 * The coeffieient used to convert a radian value into rotations of the NEO 550
 * turning motor.
 */
#define TURN_RADIAN_TO_ENCODER_FACTOR 10.1859

// The max voltage of the drive motors.
#define DRIVE_MAX_VOLTAGE 12
// The max voltage of the turning motors.
#define TURN_MAX_VOLTAGE 12

// The max amperage of the drive motors.
#define DRIVE_MAX_AMPERAGE 40
// The max amperage of the turning motors.
#define TURN_MAX_AMPERAGE 30

// The number of seconds for the drive motors to ramp from idle to full trottle.
#ifdef HOMER
# define DRIVE_RAMP_TIME 1
#else
# define DRIVE_RAMP_TIME 0.25
#endif

// The maximum speed during alignment using vision.
#define DRIVE_VISION_MAX_SPEED .1_mps

// The maximum angular speed during alignment using vision.
#define DRIVE_VISION_MAX_ANGULAR_SPEED 25_deg_per_s

// The allowable tolerance of the vision alignment.
#define VISION_TOLERANCE 0.05

// The allowable tolerance of the horizontal limelight angle.
#define LIMELIGHT_TOLERANCE 5_deg

// --- PID values ---

#define DRIVE_P_VALUE 0.00001
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 0
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 0.000187

#define ROT_P_VALUE 0.4 //0.08
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0

// --- Swerve module ---

SwerveModule::SwerveModule(ThunderSparkMax::MotorID driveID, ThunderSparkMax::MotorID turningID, int canCoderCANID, bool _driveInverted)
  : driveMotor(ThunderSparkMax::create(driveID)),
    drivePID(driveMotor->GetPIDController()),
    turningMotor(ThunderSparkMax::create(turningID)),
    turningPID(turningMotor->GetPIDController()),
    turningAbsEncoder(ThunderCANCoder::create(canCoderCANID)),
    driveInverted(_driveInverted) {
    
    configureMotors();
    
    // --- CANCoder config ---
    
    turningAbsEncoder->ConfigFactoryDefault();
    // Set the range of the CANCoder to -180 to +180 instead of 0 to 360.
    turningAbsEncoder->ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
}

SwerveModule::~SwerveModule() {

}

void SwerveModule::stop() {
    turningMotor->Set(0);
    driveMotor->Set(0);
    driveMotor->SetEncoder(0);
}

void SwerveModule::configureMotors() {
    // --- Drive motor config ---
    
    driveMotor->RestoreFactoryDefaults();

    // Set the idle mode to coast (Because the robot will start disabled).
    setIdleMode(COAST);

    // Voltage limit.
    driveMotor->EnableVoltageCompensation(DRIVE_MAX_VOLTAGE);
    // Amperage limit.
    driveMotor->SetSmartCurrentLimit(DRIVE_MAX_AMPERAGE);

    // It is not inverted!
    driveMotor->SetInverted(driveInverted);

    // Ramping (0.5 seconds to accelerate from neutral to full throttle).
    driveMotor->SetClosedLoopRampRate(DRIVE_RAMP_TIME);
    driveMotor->SetOpenLoopRampRate(DRIVE_RAMP_TIME);

    // PID Values.
    drivePID->SetP(DRIVE_P_VALUE, 0);
    drivePID->SetI(DRIVE_I_VALUE, 0);
    drivePID->SetD(DRIVE_D_VALUE, 0);
    drivePID->SetIZone(DRIVE_I_ZONE_VALUE, 0);
    drivePID->SetFF(DRIVE_FF_VALUE, 0);
  
    // --- Turning motor config ---
    
    turningMotor->RestoreFactoryDefaults();

    // Coast when idle.
    turningMotor->SetIdleMode(ThunderSparkMax::COAST);

    // Voltage limit.
    turningMotor->EnableVoltageCompensation(TURN_MAX_VOLTAGE);
    // Amperage limit.
    turningMotor->SetSmartCurrentLimit(TURN_MAX_AMPERAGE);

    // It is not inverted!
    turningMotor->SetInverted(true);

    // PID values.
    turningPID->SetP(ROT_P_VALUE, 0);
    turningPID->SetI(ROT_I_VALUE, 0);
    turningPID->SetD(ROT_D_VALUE, 0);
    turningPID->SetIZone(ROT_I_ZONE_VALUE, 0);
    turningPID->SetFF(ROT_FF_VALUE, 0);
}

void SwerveModule::doPersistentConfiguration() {
    configureMotors();
    driveMotor->BurnFlash();
    turningMotor->BurnFlash();
}

units::radian_t SwerveModule::getRawRotation() {
    return units::degree_t(turningAbsEncoder->GetAbsolutePosition());
}

double SwerveModule::getRawDriveEncoder() {
    return driveMotor->GetEncoder();
}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();
  
    // Optimize the target state by flipping motor directions and adjusting
    // rotations in order to turn the least amount of distance possible.
    frc::SwerveModuleState optimizedState;
    if (isCraterMode) {
        optimizedState = targetState;
    }
    else {
        optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    }

    // Only handle turning when we are actually driving (Stops the modules from
    // snapping back to 0 when we come to a stop).
    if(units::math::abs(optimizedState.speed) > 0.01_mps) {
        // Rotate the swerve module.
        setTurningMotor(optimizedState.angle.Radians());
    }
  
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed);
}

void SwerveModule::setDriveMotor(units::meters_per_second_t velocity) {
    // Convert meters per second into rotations per minute.
    double rpm = velocity.value() * 60 * DRIVE_METER_TO_ENCODER_FACTOR;

    // Set the target RPM of the drive motor.
    drivePID->SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
}

void SwerveModule::setTurningMotor(units::radian_t angle) {
    // Subtract the absolute rotation from the target rotation.
    units::radian_t rotation(angle - getAbsoluteRotation().Radians());
    
    // Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
    // If the value is above π rad or below -π rad...
    if(units::math::abs(rotation).value() > wpi::numbers::pi) {
        // Subtract 2π rad, or add 2π rad depending on the sign.
        rotation = units::radian_t(rotation.value() - (2 * wpi::numbers::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }

    target = rotation + 90_deg;
    
    // Convert the radian value to internal encoder value.
    double output = rotation.value() * TURN_RADIAN_TO_ENCODER_FACTOR;
    
    // Add the current relative rotation.
    output += getRelativeRotation();

    // Set PID controller reference.
    turningPID->SetReference(output, rev::CANSparkMax::ControlType::kPosition);
}

frc::SwerveModuleState SwerveModule::getState() {
    // The velocity and rotation of the swerve module.
    return { getDriveVelocity(), getAbsoluteRotation() };
}

void SwerveModule::setOffset(units::radian_t offset) {
    canCoderOffset = offset;
}

void SwerveModule::setIdleMode(IdleMode mode) {
    ThunderSparkMax::IdleMode idleMode {};
    
    switch (mode) {
        case BRAKE:
            idleMode = ThunderSparkMax::BRAKE;
            break;
        case COAST:
            idleMode = ThunderSparkMax::COAST;
            break;
    }

    // Set the idle mode of the drive motor.
    driveMotor->SetIdleMode(idleMode);
    turningMotor->SetIdleMode(idleMode);
}

units::meters_per_second_t SwerveModule::getDriveVelocity() {
    // Convert rotations per minute to meters per second.
    double mps =  (driveMotor->GetVelocity() / 60) * DRIVE_ENCODER_TO_METER_FACTOR;
    
    return units::meters_per_second_t(mps);
}

double SwerveModule::getRelativeRotation() {
    return turningMotor->GetEncoder(); 
}

frc::Rotation2d SwerveModule::getAbsoluteRotation() {
    // The angle from the CANCoder.
    units::degree_t angle(turningAbsEncoder->GetAbsolutePosition());

    // Subtract the offset from the angle.
    angle -= canCoderOffset;

    return angle;
}

// --- Drive commands ---

#define POSE_X_THRESHOLD 1_in
#define POSE_Y_THRESHOLD 1_in
#define POSE_ROT_THRESHOLD 4_deg

PetersTrajectoryController::PetersTrajectoryController() { }

PetersTrajectoryController::~PetersTrajectoryController() { }

void PetersTrajectoryController::sendFeedback() {
    const char* buffer = "";
    switch (trajectoryState) {
        case UNKNOWN:
            buffer = "unknown";
            break;
        case ACCELERATING:
            buffer = "accelerating";
            break;
        case CONSTANT:
            buffer = "constant";
            break;
        case DECELERATING:
            buffer = "decelerating";
            break;
        case ERROR_CORRECTION:
            buffer = "error correction";
            break;
    }
    Feedback::sendString("peter's trajectory controller", "trajectory state", buffer);

    Feedback::sendDouble("peter's trajectory controller", "state timer (seconds)", trajectory.timer.Get().value());
    Feedback::sendDouble("peter's trajectory controller", "start X (meters)", trajectory.start.X().value());
    Feedback::sendDouble("peter's trajectory controller", "start Y (meters)", trajectory.start.Y().value());
    Feedback::sendDouble("peter's trajectory controller", "start Rotation (degrees)", trajectory.start.Rotation().Degrees().value());
    Feedback::sendDouble("peter's trajectory controller", "end X (meters)", trajectory.end.X().value());
    Feedback::sendDouble("peter's trajectory controller", "end Y (meters)", trajectory.end.Y().value());
    Feedback::sendDouble("peter's trajectory controller", "end Rotation (degrees)", trajectory.end.Rotation().Degrees().value());
    Feedback::sendDouble("peter's trajectory controller", "max velocity (mps)", trajectory.maxVelocity.value());
    Feedback::sendDouble("peter's trajectory controller", "distance traveled (meters)", trajectory.distanceTraveled.value());
    Feedback::sendDouble("peter's trajectory controller", "total distance (meters)", trajectory.totalDistance.value());
    Feedback::sendDouble("peter's trajectory controller", "heading (degrees)", trajectory.heading.value());
    Feedback::sendDouble("peter's trajectory controller", "accelerate distance (meters)", trajectory.accelerateDistance.value());
    Feedback::sendDouble("peter's trajectory controller", "decelerate distance (meters)", trajectory.decelerateDistance.value());
    Feedback::sendDouble("peter's trajectory controller", "constant distance (meters)", trajectory.constantDistance.value());
    Feedback::sendDouble("peter's trajectory controller", "configured max velocity", trajectory.config.maxVelocity.value());
    Feedback::sendDouble("peter's trajectory controller", "configured max acceleration", trajectory.config.maxAcceleration.value());
    Feedback::sendDouble("peter's trajectory controller", "configured max angular velocity", trajectory.config.maxAngularVelocity.value());
    Feedback::sendDouble("peter's trajectory controller", "configured start velocity", trajectory.config.startVelocity.value());
    Feedback::sendDouble("peter's trajectory controller", "configured end velocity", trajectory.config.endVelocity.value());
}

void PetersTrajectoryController::setTrajectory(frc::Pose2d currentPose, frc::Pose2d endPose, PetersTrajectoryConfig config) {
    trajectory = {};

    trajectory.start = currentPose;
    trajectory.end = endPose;
    trajectory.config = config;

    units::meter_t xDistance = trajectory.end.X() - trajectory.start.X();
    units::meter_t yDistance = trajectory.end.Y() - trajectory.start.Y();

    // Get the total distance to travel.
    trajectory.totalDistance = units::math::sqrt(units::math::pow<2>(xDistance) + units::math::pow<2>(yDistance));

    // Find the maximum velocity possible.
    trajectory.maxVelocity = units::meters_per_second_t(std::sqrt((2 * trajectory.config.maxAcceleration.value() * trajectory.totalDistance.value() + std::pow(trajectory.config.startVelocity.value(), 2) + std::pow(trajectory.config.endVelocity.value(), 2)) / 2));

    if (units::math::abs(trajectory.maxVelocity) > units::math::abs(trajectory.config.maxVelocity)) {
        trajectory.maxVelocity = trajectory.config.maxVelocity;
    }

    // Get the distance to accelerate to max velocity.
    trajectory.accelerateDistance = (units::math::pow<2>(trajectory.maxVelocity) - units::math::pow<2>(trajectory.config.startVelocity)) / (2 * trajectory.config.maxAcceleration);
    // Get the distance to decelerate to final velocity.
    trajectory.decelerateDistance = (units::math::pow<2>(trajectory.config.endVelocity) - units::math::pow<2>(trajectory.maxVelocity)) / (2 * -trajectory.config.maxAcceleration);
    // Get the remaining distance in between during which the robot is at max velocity.
    trajectory.constantDistance = trajectory.totalDistance - trajectory.accelerateDistance - trajectory.decelerateDistance;

    // Get the angle the robot will be moving.
    trajectory.heading = units::math::atan2(yDistance, xDistance);
    trajectoryState = ACCELERATING;
    trajectory.timer.Reset();
    trajectory.timer.Start();
}

bool PetersTrajectoryController::atReference(frc::Pose2d currentPose) {
    if (trajectoryState == UNKNOWN) {
        return true;
    }
    
    if (units::math::abs(trajectory.end.X() - currentPose.X()) <= POSE_X_THRESHOLD &&
        units::math::abs(trajectory.end.Y() - currentPose.Y()) <= POSE_Y_THRESHOLD &&
        units::math::abs(trajectory.end.Rotation().Degrees() - currentPose.Rotation().Degrees()) <= POSE_ROT_THRESHOLD) {
        trajectoryState = UNKNOWN;
        return true;
    }

    return false;
}

frc::ChassisSpeeds PetersTrajectoryController::getVelocities(frc::Pose2d currentPose) {
    if (atReference(currentPose)) {
        return { 0_mps, 0_mps, 0_rad_per_s };
    }

    units::meter_t xDistance = currentPose.X() - trajectory.start.X();
    units::meter_t yDistance = currentPose.Y() - trajectory.start.Y();

    // Get the total distance traveled.
    trajectory.distanceTraveled = units::math::sqrt(units::math::pow<2>(xDistance) + units::math::pow<2>(yDistance));

    TrajectoryState lastState = trajectoryState;

    switch (trajectoryState) {
        case UNKNOWN:
            break;
        case ACCELERATING:
            if (trajectory.distanceTraveled > trajectory.accelerateDistance) {
                trajectoryState = CONSTANT;
            }
            break;
        case CONSTANT:
            if (trajectory.distanceTraveled > trajectory.accelerateDistance + trajectory.constantDistance) {
                trajectoryState = DECELERATING;
            }
            break;
        case DECELERATING:
            if (units::math::abs(trajectory.distanceTraveled - trajectory.totalDistance) < 0.1_m) {
                trajectoryState = ERROR_CORRECTION;
            }
            break;
        case ERROR_CORRECTION:
            break;
    }

    if (trajectoryState != lastState) {
        trajectory.timer.Reset();
        trajectory.timer.Start();
    }

    units::meters_per_second_t vel = 0_mps;
    units::radians_per_second_t angVel = 0_rad_per_s;

    if (trajectory.end.Rotation().Degrees() - currentPose.Rotation().Degrees() > POSE_ROT_THRESHOLD) {
        angVel = DRIVE_CMD_MAX_ANGULAR_VELOCITY;
    }
    else if (trajectory.end.Rotation().Degrees() - currentPose.Rotation().Degrees() < -POSE_ROT_THRESHOLD) {
        angVel = -DRIVE_CMD_MAX_ANGULAR_VELOCITY;
    }
    
    // std::cout << angVel.value() << '\n';

    // angVel = units::radians_per_second_t(trajectory.rotError.Calculate())

    switch (trajectoryState) {
        case UNKNOWN:
            std::cout << "unknown\n";
            vel = 0_mps;
            angVel = 0_rad_per_s;
            break;
        case ACCELERATING:
            std::cout << "accelerating\n";
            vel = (trajectory.config.maxAcceleration * trajectory.timer.Get())
                  * (std::signbit(trajectory.totalDistance.value()) ? -1 : 1);
            break;
        case CONSTANT:
            std::cout << "constant\n";
            vel = trajectory.maxVelocity
                  * (std::signbit(trajectory.totalDistance.value()) ? -1 : 1);
            break;
        case DECELERATING:
            std::cout << "decelerating\n";
            vel = (trajectory.maxVelocity - (trajectory.config.maxAcceleration * trajectory.timer.Get()))
                  * (std::signbit(trajectory.totalDistance.value()) ? -1 : 1);
            break;
        case ERROR_CORRECTION:
            std::cout << "error correction\n";
            break;
    }

    units::meters_per_second_t xVel = vel * units::math::cos(trajectory.heading);
    units::meters_per_second_t yVel = vel * units::math::sin(trajectory.heading);

    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, angVel, currentPose.Rotation());
}

// --- Drivetrain ---

Drive::Drive(Camera* camera, Limelight* limelight)
  : camera(camera), limelight(limelight), imu(NULL) {

    // thetaController.EnableContinuousInput(-180_deg, 180_deg);
    // cmdController.SetEnabled(true);

#ifndef TEST_BOARD
    imu = new frc::ADIS16470_IMU();
    // Configure the calibration time of the IMU to 4 seconds.
    imu->ConfigCalTime(frc::ADIS16470_IMU::CalibrationTime::_4s);
    // Set the default axis for the IMU's gyro to take.
#ifdef HOMER
    imu->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
#else
    imu->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kY);
#endif
#endif

    // Zero the IMU.
    resetIMU();
#ifdef HOMER
    // Calibrate the IMU.
    calibrateIMU();
#endif

    // Read encoder offsets from the file and apply them to the swerve modules.
    if (readOffsetsFile()) {
        applyOffsets();
    }
}

Drive::~Drive() {
    // Free the swerve module objects from heap memory.
    for (SwerveModule* module : swerveModules) {
        delete module;
    }
    delete imu;
}

void Drive::doPersistentConfiguration() {
    for (SwerveModule* module : swerveModules)
        module->doPersistentConfiguration();

    configMagneticEncoders();
}

void Drive::resetToMode(MatchMode mode) {
    // Cancel the current command.
    cmdCancel();

    driveMode = STOPPED;
    cmdType = NONE;
    trajectoryController = {};
    manualData = {};

    for(SwerveModule* module : swerveModules) {
      module->stop();
    }
    
    if (mode == MODE_DISABLED) {
        // Set the drive motors to coast when disabled so they can try to push
        // swerve drive (not going to happen but ok).
        setIdleMode(SwerveModule::COAST);
    }
    else {
        resetOdometry();

        // Set the drive motors to brake in teleop and autonomous modes because
        // we want the robot to stop when we tell it to stop.
        setIdleMode(SwerveModule::BRAKE);
    }
}

void Drive::sendFeedback() {
    trajectoryController.sendFeedback();

    Feedback::sendDouble("thunderdashboard", "gyro", !imuCalibrated);

    Feedback::sendDouble("drive", "imu angle (degrees)", imu ? imu->GetAngle().value() : 0);

    const char* buffer = "";

    switch (driveMode) {
        case STOPPED:
            buffer = "stopped";
            break;
        case MANUAL:
            buffer = "manual";
            break;
        case COMMAND:
            buffer = "command";
            break;
    }
    Feedback::sendString("drive", "drive mode", buffer);

    switch (controlMode) {
        case FIELD_CENTRIC:
            buffer = "field centric";
            break;
        case ROBOT_CENTRIC:
            buffer = "robot centric";
            break;
    }
    Feedback::sendString("drive", "control mode", buffer);

    switch (cmdType) {
        case NONE:
            buffer = "none";
            break;
        case TRAJECTORY:
            buffer = "trajectory";
            break;
        case ALIGN_TO_CARGO:
            buffer = "align with cargo";
            break;
        case ALIGN_TO_HIGH_HUB:
            buffer = "align with high hub";
            break;
    }
    Feedback::sendString("drive", "command type", buffer);
    
    frc::Pose2d pose = getPose();

    Feedback::sendDouble("drive", "robot x position (meters)", pose.X().value());
    Feedback::sendDouble("drive", "robot y position (meters)", pose.Y().value());
    Feedback::sendDouble("drive", "robot rotation (degrees)", getRotation().Degrees().value());

    Feedback::sendDouble("drive", "wheel 0 rotation (degrees)", swerveModules.at(0)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 1 rotation (degrees)", swerveModules.at(1)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 2 rotation (degrees)", swerveModules.at(2)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 3 rotation (degrees)", swerveModules.at(3)->getState().angle.Degrees().value());

    Feedback::sendDouble("drive", "wheel 0 rotation without offsets (radians)", swerveModules.at(0)->getState().angle.Radians().value() + offsets.at(0).value());
    Feedback::sendDouble("drive", "wheel 1 rotation without offsets (radians)", swerveModules.at(1)->getState().angle.Radians().value() + offsets.at(1).value());
    Feedback::sendDouble("drive", "wheel 2 rotation without offsets (radians)", swerveModules.at(2)->getState().angle.Radians().value() + offsets.at(2).value());
    Feedback::sendDouble("drive", "wheel 3 rotation without offsets (radians)", swerveModules.at(3)->getState().angle.Radians().value() + offsets.at(3).value());

    Feedback::sendDouble("drive", "wheel 0 offset", offsets.at(0).value());
    Feedback::sendDouble("drive", "wheel 1 offset", offsets.at(1).value());
    Feedback::sendDouble("drive", "wheel 2 offset", offsets.at(2).value());
    Feedback::sendDouble("drive", "wheel 3 offset", offsets.at(3).value());

    Feedback::sendDouble("drive", "module 0 target rotation (degerees)", swerveModules.at(0)->target.value());
    Feedback::sendDouble("drive", "module 1 target rotation (degerees)", swerveModules.at(1)->target.value());
    Feedback::sendDouble("drive", "module 2 target rotation (degerees)", swerveModules.at(2)->target.value());
    Feedback::sendDouble("drive", "module 3 target rotation (degerees)", swerveModules.at(3)->target.value());
    
    Feedback::sendDouble("drive", "module 0 drive encoder", swerveModules.at(0)->getRawDriveEncoder());
    Feedback::sendDouble("drive", "module 1 drive encoder", swerveModules.at(1)->getRawDriveEncoder());
    Feedback::sendDouble("drive", "module 2 drive encoder", swerveModules.at(2)->getRawDriveEncoder());
    Feedback::sendDouble("drive", "module 3 drive encoder", swerveModules.at(3)->getRawDriveEncoder());

    //hi josh

    Feedback::sendDouble("drive", "wheel 0 speed (mps)", swerveModules.at(0)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 1 speed (mps)", swerveModules.at(1)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 2 speed (mps)", swerveModules.at(2)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 3 speed (mps)", swerveModules.at(3)->getState().speed.value());

    Feedback::sendDouble("drive", "manual X pct", manualData.xPct);
    Feedback::sendDouble("drive", "manual Y pct", manualData.yPct);
    Feedback::sendDouble("drive", "manual angular pct", manualData.rotPct);
    Feedback::sendBoolean("drive", "vice grip", manualData.viceGrip);
}

void Drive::process() {
    // hi nevin
    // hi jeff :D
    
    // Continue tracking the position and rotation of the robot on the field.
    updateOdometry();

    switch (driveMode) {
        case STOPPED:
            // Stop all the swerve modules.
            setModuleStates({ 0_mps, 0_mps, 0_rad_per_s });
            break;
        case MANUAL:
            // Execute the manual instructions.
            exeManual();
            break;
        case COMMAND:
            if (cmdIsFinished()) {
                cmdType = NONE;
            }

            switch (cmdType) {
                case NONE:
                    // Stop the command.
                    driveMode = STOPPED;
                    break;
                case TRAJECTORY:
                    // Execute the follow trajectory command.
                    exeFollowTrajectory();
                    break;
                case ALIGN_TO_CARGO:
                    // Execute the align with cargo command.
                    exeAlignToCargo();
                    break;
                case ALIGN_TO_HIGH_HUB:
                    // Execute the align with high hub command.
                    exeAlignToHighHub();
                    break;
            }
            break;
    }
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

void Drive::zeroRotation() {
    resetIMU();
    updateOdometry();
}

void Drive::calibrateIMU() {
    if (imu) {
        imu->Calibrate();
        sleep(4);
        imuCalibrated = true;
    }
}

void Drive::configMagneticEncoders() {
    if (!isCraterMode) {
        return;
    }

    // Apply the current rotation of the swerve modules to the offsets.
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        units::radian_t angle(swerveModules.at(i)->getRawRotation());

        // Add the angle of the module to the offset that is already applied.
        offsets.at(i) = angle;
    }
    
    // Write the new offsets to the offsets file.
    writeOffsetsFile();

    // Apply the new offsets to the swerve modules.
    applyOffsets();
}

void Drive::manualDrive(double xPct, double yPct, double rotPct) {
    if (xPct == 0 && yPct == 0 && rotPct == 0 && !manualData.viceGrip) {
        if (driveMode != COMMAND) {
            driveMode = STOPPED;
        }
    }
    else {
        driveMode = MANUAL;
    }

    bool vg = manualData.viceGrip;

    manualData = { xPct, yPct, rotPct, vg };
}

void Drive::setControlMode(ControlMode mode) {
    controlMode = mode;
}

void Drive::makeBrick() {
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        units::degree_t angle;
        // If the index is even.
        if (i % 2 == 0) {
            angle = -45_deg;
        }
        // If the index is odd.
        else {
            angle = 45_deg;
        }
        // Stop the robot.
        driveMode = STOPPED;
        // Turn the swerve module to point towards the center of the robot.
        swerveModules.at(i)->setTurningMotor(angle);
    }
}

void Drive::setViceGrip(bool viceGrip) {
    manualData.viceGrip = viceGrip;
}

frc::TrajectoryConfig Drive::getTrajectoryConfig() {
    // Generate a configuration with speed and acceleration limits.
    frc::TrajectoryConfig trajectoryConfig { DRIVE_CMD_MAX_VELOCITY, DRIVE_CMD_MAX_ACCELERATION };

    // Add a constraint to the configuration to make the trajectory better
    // suited for a swerve drive.
    trajectoryConfig.AddConstraint(frc::SwerveDriveKinematicsConstraint<4>(kinematics, DRIVE_CMD_MAX_VELOCITY));

    // Stupid class does not like copy constructors for some reason.
    return std::move(trajectoryConfig);
}

bool Drive::cmdAlignToCargo() {
    // Make sure it is in auto and that the camera sees a target.
    if (getCurrentMode() != MODE_AUTO || !camera->hasTarget()) {
        return false;
    }

    driveMode = COMMAND;
    cmdType = ALIGN_TO_CARGO;

    return true;
}

bool Drive::cmdAlignToHighHub() {
    // Make sure the limelight sees a target.
    if (!limelight->hasTarget()) {
        return false;
    }

    driveMode = COMMAND;
    cmdType = ALIGN_TO_HIGH_HUB;
    alignmentData.position = AlignmentData::UNKNOWN;
    
    return true;
}

void Drive::cmdRotateToAngle(frc::Rotation2d angle, units::radians_per_second_t velocity) {
    frc::Pose2d currentPose = getPose();

    PetersTrajectoryConfig config {};
    config.maxAngularVelocity = velocity;

    cmdDriveToPose(currentPose.X(), currentPose.Y(), angle, config);
}

void Drive::cmdDriveTranslate(units::meter_t x, units::meter_t y, frc::Rotation2d angle, PetersTrajectoryConfig config) {
    frc::Pose2d currentPose = getPose();

    frc::Pose2d targetPose = { currentPose.X() + x, currentPose.Y() + y, angle };

    cmdDriveToPose(targetPose.X(), targetPose.Y(), targetPose.Rotation(), config);
}

void Drive::cmdDriveToPose(units::meter_t x, units::meter_t y, frc::Rotation2d angle, PetersTrajectoryConfig config) {
    driveMode = COMMAND;
    cmdType = TRAJECTORY;

    trajectoryController.setTrajectory(getPose(), { x, y, units::math::fmod(angle.Degrees(), 360_deg) }, config);
}

bool Drive::cmdIsFinished() {
    // If no command is running, then it has finished.
    if (driveMode != COMMAND) {
        return true;
    }

    switch (cmdType) {
        // If no command is running, then it has finished.
        case NONE:
            return true;
        case TRAJECTORY:
            if (trajectoryController.atReference(getPose())) {
                std::cout << "WE ARE FINISHED!!\n";
                return true;
            }
            return false;
        case ALIGN_TO_CARGO:
            // Check if the cargo is in the center of the frame.
            if (camera->getTargetSector() == Camera::CENTER) {
                return true;
            }
            return false;
        case ALIGN_TO_HIGH_HUB:
            if (alignmentData.position == AlignmentData::CENTER) {
                alignmentData.position = AlignmentData::UNKNOWN;
                return true;
            }
            return false;
    }

    return true;
}

void Drive::cmdCancel() {
    cmdType = NONE;
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
    // Generate module states using the chassis velocities. This is the magic
    // function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
    
    // Recalculate the wheel velocities relative to the max speed.
    kinematics.DesaturateWheelSpeeds(&moduleStates, DRIVE_MANUAL_MAX_VELOCITY);
    
    // Set the module states.
    for(unsigned i = 0; i < swerveModules.size(); i++) {
      swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

void Drive::updateOdometry() {
    // Update the position and rotation on the field.
    odometry.Update(getRotation(),
        swerveModules.at(0)->getState(),
        swerveModules.at(1)->getState(),
        swerveModules.at(2)->getState(),
        swerveModules.at(3)->getState());
}

void Drive::resetOdometry(frc::Pose2d pose) {
    // Reset the position on the field.
    odometry.ResetPosition(pose, pose.Rotation());
}

void Drive::resetIMU() {
    if (imu)
        imu->Reset();
}

frc::Rotation2d Drive::getRotation() {
    units::angle::degree_t imuAngle = 0_deg;
    if (imu)
        imuAngle = imu->GetAngle();

    // Get the current rotation of the robot.
    units::radian_t rotation(units::math::fmod(imuAngle, 360_deg));

    // Convert -2π to 2π value into -π to π value.
    if(units::math::abs(rotation).value() > wpi::numbers::pi) {
        rotation = units::radian_t(rotation.value() - (2 * wpi::numbers::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    return frc::Rotation2d(rotation);
}

void Drive::exeManual() {
    if (!imuCalibrated) {
        calibrateIMU();
    }

    // Calculate the velocities.
    units::meters_per_second_t xVel    = manualData.xPct   * DRIVE_MANUAL_MAX_VELOCITY;
    units::meters_per_second_t yVel    = manualData.yPct   * DRIVE_MANUAL_MAX_VELOCITY;
    units::radians_per_second_t rotVel;
    
    if (manualData.viceGrip) {
        rotVel = getAlignVelocity();
    }
    else {
        rotVel = manualData.rotPct * DRIVE_MANUAL_MAX_ANGULAR_VELOCITY;
    }

    frc::ChassisSpeeds chassisVelocities;

    // Generate chassis speeds depending on the control mode.
    switch (controlMode) {
        case FIELD_CENTRIC:
            chassisVelocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, rotVel, getRotation());
            break;
        case ROBOT_CENTRIC:
            chassisVelocities = { xVel, yVel, rotVel };
            break;
    }
    
    // Set the modules to drive based on the velocities.
    setModuleStates(chassisVelocities);
}

void Drive::exeFollowTrajectory() {
    // Calculate chassis velocities that are required in order to reach the
    // desired state.
    frc::ChassisSpeeds targetChassisSpeeds = trajectoryController.getVelocities(getPose());

    std::cout << "vel " << targetChassisSpeeds.vx.value() << " " << targetChassisSpeeds.vy.value() << " " << targetChassisSpeeds.omega.value() << '\n';

    // Drive!
    setModuleStates(targetChassisSpeeds);

    //hi ishan
    //hi jeff
    //hi trevor
    //hi nevin
    //hi josh
    //hi byers
    //hi calla
    //hi nadia
    //hi homer 2.0
}

void Drive::exeAlignToCargo() {
    switch (camera->getTargetSector()) {
        case Camera::UNKNOWN:
            // Not found.
            cmdCancel();
            break;
        case Camera::CENTER:
            // Found in the center.
            break;
        case Camera::LEFT:
            // Begin rotating the the left.
            setModuleStates({ 0_mps, 0_mps, -DRIVE_VISION_MAX_ANGULAR_SPEED });
            break;
        case Camera::RIGHT:
            // Begin rotating to the right.
            setModuleStates({ 0_mps, 0_mps, +DRIVE_VISION_MAX_ANGULAR_SPEED });
            break;
    }
}

void Drive::exeAlignToHighHub() {
    setModuleStates({0_mps, 0_mps, getAlignVelocity() });
}

bool Drive::readOffsetsFile() {
    // Open the file.
    std::ifstream file(ENCODER_OFFSETS_FILE_NAME);
    // Make sure the file exists.
    if (!file) {
        return false;
    }

    unsigned i = 0;
    std::string line;
    // Loop through each line of the file.
    while (getline(file, line) && i <= 3) {
        // Convert the line to a number.
        double num = std::atof(line.c_str());
        
        // Make sure the number is not 0 (We don't really care about 0s, plus
        // atof() returns 0 when there is an error parsing the string)
        if (num) {
            // Set the offset in the array to the parsed number.
            offsets.at(i) = units::radian_t(num);
        }
        // Increment the index.
        i++;
    }

    return true;
    //hi nadia
}

void Drive::writeOffsetsFile() {
    // Open the file (Will create a new file if it does not already exist).
    std::ofstream file(ENCODER_OFFSETS_FILE_NAME);
    // Clear the contents of the file.
    file.clear();

    // Write each offset to the file.
    for (units::radian_t offset : offsets) {
        file << offset.value() << '\n';
    }
}

void Drive::applyOffsets() {
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->setOffset(offsets.at(i) - 90_deg);
    }
}

void Drive::setIdleMode(SwerveModule::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}

units::radians_per_second_t Drive::getAlignVelocity() {
    if (limelight->hasTarget()) {
        units::radian_t angle = limelight->getAngleHorizontal();

        if (units::math::abs(angle) <= LIMELIGHT_TOLERANCE) {
            alignmentData.position = AlignmentData::CENTER;
        }
        else if (angle > LIMELIGHT_TOLERANCE) {
            alignmentData.position = AlignmentData::RIGHT;
        }
        else if (angle < -LIMELIGHT_TOLERANCE) {
            alignmentData.position = AlignmentData::LEFT;
        }
    }

    switch (alignmentData.position) {
        case AlignmentData::UNKNOWN:
            return 0_rad_per_s;
        case AlignmentData::CENTER:
            return 0_rad_per_s;
        case AlignmentData::RIGHT:
            return -DRIVE_VISION_MAX_ANGULAR_SPEED;
        case AlignmentData::LEFT:
            return +DRIVE_VISION_MAX_ANGULAR_SPEED;
    }

    return 0_rad_per_s;
}