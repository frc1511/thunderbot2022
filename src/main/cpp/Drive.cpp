#include "Drive.h"
#include <cmath>
#include <vector>

// The circumference of each wheel (meters).
#define WHEEL_CIRCUMFERENCE 0.21

// The value of the encoder after 1 rotation.
#define WHEEL_1_ROT_ENC_VAL 1

// The coefficient used to convert the internal drive encoder value to meters (Drive motor).
#define DRIVE_ENC_TO_METERS_FACTOR (WHEEL_CIRCUMFERENCE / WHEEL_1_ROT_ENC_VAL)

// The coeffieient used to convert a radian value to internal encoder value (Turning motor).
#define ROT_RAD_TO_ENC_FACTOR 10.1859

#define DRIVE_MAX_VOLTAGE 12
#define TURNING_MAX_VOLTAGE 12

// --- PID values ---

#define DRIVE_P_VALUE 1
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 0
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 0

#define ROT_P_VALUE 0.4
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0

// --- Swerve module ---

SwerveModule::SwerveModule(int driveCANID, int turningCANID, int canCoderCANID, double canCoderOffset)
  : driveMotor(driveCANID, rev::CANSparkMax::MotorType::kBrushless),
    driveEncoder(driveMotor.GetEncoder()),
    drivePID(driveMotor.GetPIDController()),
    turningMotor(turningCANID, rev::CANSparkMax::MotorType::kBrushless),
    turningRelEncoder(turningMotor.GetEncoder()),
    turningPID(turningMotor.GetPIDController()),
    turningAbsEncoder(canCoderCANID),
    canCoderOffset(canCoderOffset) {
  
    // --- Drive motor config ---
    
    driveMotor.RestoreFactoryDefaults();
    // Brake when idle.
    driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor.EnableVoltageCompensation(DRIVE_MAX_VOLTAGE);
    // Limit in amps (Always when using NEO Brushless to avoid damage).
    driveMotor.SetSmartCurrentLimit(40);
    driveMotor.SetInverted(false);
    // Ramping (0.5 seconds to accelerate from neutral to full throttle).
    driveMotor.SetClosedLoopRampRate(0.5);
    driveMotor.SetOpenLoopRampRate(0.5);
    // Frame period.
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);
    // Apply a coefficient to convert the encoder value to meters.
    driveEncoder.SetPositionConversionFactor(DRIVE_ENC_TO_METERS_FACTOR);
    // PID Values.
    drivePID.SetP(DRIVE_P_VALUE, 0);
    drivePID.SetI(DRIVE_I_VALUE, 0);
    drivePID.SetD(DRIVE_D_VALUE, 0);
    drivePID.SetIZone(DRIVE_I_ZONE_VALUE, 0);
    drivePID.SetFF(DRIVE_FF_VALUE, 0);
  
    // --- Turning motor config ---
    
    turningMotor.RestoreFactoryDefaults();
    turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    turningMotor.EnableVoltageCompensation(TURNING_MAX_VOLTAGE);
    turningMotor.SetSmartCurrentLimit(30);
    turningMotor.SetInverted(true);
    turningPID.SetFeedbackDevice(turningRelEncoder);
    // PID values.
    turningPID.SetP(ROT_P_VALUE, 0);
    turningPID.SetI(ROT_I_VALUE, 0);
    turningPID.SetD(ROT_D_VALUE, 0);
    turningPID.SetIZone(ROT_I_ZONE_VALUE, 0);
    turningPID.SetFF(ROT_FF_VALUE, 0);
    
    // --- CANCoder config ---
    
    turningAbsEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
}

SwerveModule::~SwerveModule() {

}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();
  
    // Optimize the target state by flipping motor directions and adjusting rotations in order to turn the least amount of distance possible.
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    
    // Only handle turning when we are actually driving.
    if(units::math::abs(optimizedState.speed) > 0.01_mps) {
        // Rotate the swerve module.
        setTurningMotor(optimizedState.angle.Radians());
    }
  
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed.value());
}

frc::SwerveModuleState SwerveModule::getState() {
    return { units::meters_per_second_t(getDriveVelocity()), frc::Rotation2d(getAbsoluteRotation()) };
}

void SwerveModule::setDriveMotor(double speed) {
    driveMotor.Set(speed);
}

double SwerveModule::getDriveVelocity() {
    return driveEncoder.GetVelocity();
}

void SwerveModule::setTurningMotor(units::radian_t angle) {
    // Subtract the absolute rotation from the target rotation.
    units::radian_t rotation(angle - getAbsoluteRotation());
    
    // Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
    // If the value is above π rad or below -π rad...
    if(units::math::abs(rotation).value() > wpi::numbers::pi) {
        // Subtract 2π rad, or add 2π rad depending on the sign.
        rotation = units::radian_t(rotation.value() - (2 * wpi::numbers::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    // Convert the radian value to internal encoder value.
    double output = rotation.value() * ROT_RAD_TO_ENC_FACTOR;
    
    // Add the current relative rotation.
    output += getRelativeRotation();
    
    // Set PID controller reference.
    // turningPID.SetReference(output, rev::ControlType::kPosition);
    turningPID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);
}

double SwerveModule::getRelativeRotation() {
    return turningRelEncoder.GetPosition();
}

units::radian_t SwerveModule::getAbsoluteRotation() {
    return units::radian_t(units::degree_t(turningAbsEncoder.GetAbsolutePosition())) - canCoderOffset;
}

// --- Drivetrain ---

Drive::Drive(Limelight* limelight)
  : limelight(limelight) {
    
    resetIMU();
    // Configure the calibration time to 4 seconds.
    imu.ConfigCalTime(frc::ADIS16470_IMU::CalibrationTime::_4s);
    // Set axis for the gryo to take (Z is up and down).
    imu.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
    // Calibrate the IMU.
    calibrateIMU();
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
        delete module;
    }
}

void Drive::resetToMode(MatchMode mode) {
    cmdCancel();
}

void Drive::sendFeedback() {
    // TODO Send feedback...
}

void Drive::process() {
    // Update the position on the field.
    updateOdometry();

    if (command && !command.value().IsFinished()) {
        command.value().Execute();
    }
}

frc::Rotation2d Drive::getRotation() {
    units::radian_t rotation = units::radian_t(fmod(imu.GetAngle().value(), 360));

    // Convert -2π to 2π value into -π to π value.
    if(units::math::abs(rotation).value() > wpi::numbers::pi) {
        rotation = units::radian_t(rotation.value() - (2 * wpi::numbers::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    return frc::Rotation2d(units::degree_t(rotation));
}

void Drive::zeroRotation() {
    resetIMU();
}

void Drive::calibrateIMU() {
    imu.Calibrate();
}

void Drive::manualDrive(double xVel, double yVel, double rotVel, bool fieldCentric) {
    // Take control if drive command is running.
    cmdCancel();
    
    // Generate chassis speeds depending on control mode.
    if (fieldCentric) {
        setModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel), getRotation()));
    } else {
        setModuleStates({ units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel) });
    }
}

bool Drive::cmdRotateToCargo() {
    
    // TODO Implement.
    
    return true;
}

bool Drive::cmdRotateToHub() {
    // Make sure the limelight sees a target.
    if (!limelight->hasTarget()) return false;
    
    // Get the horizontal angle from limelight.
    units::radian_t angle = limelight->getAngleHorizontal();

    // Start a rotate command.
    cmdRotate(angle);

    return true;
}

void Drive::cmdRotate(frc::Rotation2d angle) {
    // Create a trajectory to rotate the specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { 0_m, 0_m, angle }, AUTO_TRAJECTORY_CONFIG);

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle) {
    // Create a trajectory that drives the specified distance / turns the specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { x, y, angle }, AUTO_TRAJECTORY_CONFIG);

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdFollowPathWeaverTrajectory(const char* pathweaverJson) {
    fs::path directory = fs::path(frc::filesystem::GetDeployDirectory())/pathweaverJson;
    // Load trajectory from Pathweaver json file.
    cmdFollowTrajectory(frc::TrajectoryUtil::FromPathweaverJson(directory.string()));
}

void Drive::cmdFollowTrajectory(frc::Trajectory trajectory) {
    // Create a controller that will follow the specified trajectory.
    command = frc2::SwerveControllerCommand<4>(
        // The trajectory for the controller follow.
        trajectory,
        
        // A lambda function to get the current pose of the robot at any given
        // time during the command's execution.
        std::function<frc::Pose2d()>([this]() -> frc::Pose2d {
            return this->getPose();
        }),
        
        // The swerve drive kinematics.
        kinematics,
        
        // PID loop for movement in the X direction.
        frc2::PIDController(1, 0, 0),
        
        // PID loop for movement in the Y direction.
        frc2::PIDController(1, 0, 0),
        
        // PID loop for rotational movement.
        frc::ProfiledPIDController<units::radians>(1, 0, 0, {}),
        
        // A lambda function to set the swerve module states at any given time
        // during the command's execution.
        std::function<void(std::array<frc::SwerveModuleState, 4>)>([this](std::array<frc::SwerveModuleState, 4> states){
            for(unsigned i = 0; i < this->swerveModules.size(); i++) {
                this->swerveModules.at(i)->setState(states.at(i));
            }
        })
    );

    // Initialize the trajectory.
    command.value().Initialize();
}

bool Drive::cmdIsFinished() {
    return command ? command.value().IsFinished() : true;
}

void Drive::cmdCancel() {
    if (command) {
        command.value().Cancel();
        command = {};
    }
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
    // Generate module states using chassis velocities. The magic function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
    
    // Recalculate wheel speeds relative to the max speed.
    kinematics.DesaturateWheelSpeeds(&moduleStates, DRIVE_MAX_SPEED);
    
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
    odometry.ResetPosition(pose, getRotation());
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

void Drive::resetIMU() {
    imu.Reset();
}