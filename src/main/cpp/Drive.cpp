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

Drive::Drive() {
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

void Drive::process() {
    // Update the position on the field.
    updateOdometry();
    
    // If a drive command is executing.
    if (cmdRunning) {
        // If the drive command has finished.
        if (cmdTimer.HasElapsed(cmdTargetTrajectory.TotalTime())) {
            cmdCancel();
        }
        // Execute the drive command.
        else {
            // Get the current time of the trajectory.
            units::second_t curTime = cmdTimer.Get();
            // Sample the state at the current time.
            frc::Trajectory::State state = cmdTargetTrajectory.Sample(curTime);
            // Set the swerve modules.
            setModuleStates(cmdController.Calculate(getPose(), state, state.pose.Rotation()));
        }
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

void Drive::resetForward() {
    resetIMU();
}

void Drive::calibrateIMU() {
    imu.Calibrate();
}

void Drive::setVelocities(double xVel, double yVel, double rotVel, bool fieldCentric) {
    // Take control if drive command is running.
    cmdCancel();
    
    // Generate chassis speeds depending on control mode.
    if (fieldCentric) {
        setModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel), getRotation()));
    } else {
        setModuleStates({ units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel) });
    }
}

void Drive::cmdRotate(frc::Rotation2d angle) {
    // Cancel a command if running.
    cmdCancel();
    cmdRunning = true;

    // Get the current pose of the robot.
    frc::Pose2d pose = getPose();

    // Add displacement to current pose.
    std::vector<frc::Trajectory::State> states { { 0_s, 0_mps, 0_mps_sq, { pose.X(), pose.Y(), pose.Rotation() + angle }, {} } };
    
    cmdTargetTrajectory = frc::Trajectory(states);
    
    // Start the timer.
    cmdTimer.Reset();
    cmdTimer.Start();
}

void Drive::cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle) {
    // Cancel a command if running.
    cmdCancel();
    cmdRunning = true;
    
    // Get the current pose of the robot.
    frc::Pose2d pose = getPose();

    // Add displacement to current pose.
    std::vector<frc::Trajectory::State> states { { 0_s, 0_mps, 0_mps_sq, { pose.X() + x, pose.Y() + y, pose.Rotation() + angle }, {} } };
    
    cmdTargetTrajectory = frc::Trajectory(states);
    
    // Start the timer.
    cmdTimer.Reset();
    cmdTimer.Start();
}

void Drive::cmdTrajectory(frc::Trajectory trajectory) {
    // Cancel a command if running.
    cmdCancel();
    cmdRunning = true;
    
    cmdTargetTrajectory = trajectory;
    
    // Start the timer.
    cmdTimer.Reset();
    cmdTimer.Start();
}

bool Drive::cmdIsFinished() {
    return !cmdRunning;
}

void Drive::cmdCancel() {
    if (!cmdRunning) return;
    
    cmdRunning = false;
    cmdTimer.Stop();
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
    // Generate module states using chassis velocities. The magic function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
    
    // Recalculate wheel speeds relative to the max speed.
    kinematics.DesaturateWheelSpeeds(&moduleStates, MAX_SPEED);
    
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