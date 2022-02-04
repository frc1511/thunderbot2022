#include "Drive.h"

// The path to the file to save encoder offsets into.
#define ENCODER_OFFSETS_FILE_NAME "/home/lvuser/magnetic_encoder_offsets.txt"

// The circumference of the drive wheels (meters).
#define DRIVE_WHEEL_CIRCUMFERENCE 0.21

/**
 * The gear ratio of the drive motor (5.25:1).
 * 
 * AKA: The number of rotations the drive encoder will read after 1 rotation of
 * the wheel.
 */
#define DRIVE_GEAR_RATIO 5.25

/**
 * The coefficient used to convert rotations of the NEO motors into a distance
 * traveled in meters.
 */
#define DRIVE_ENCODER_TO_METER_FACTOR (DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO)

/**
 * The coefficient used to convert a distance in meters into a number of
 * rotations of the NEO motors.
 */
#define DRIVE_METER_TO_ENCODER_FACTOR (DRIVE_GEAR_RATIO / DRIVE_WHEEL_CIRCUMFERENCE)

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
#define DRIVE_RAMP_TIME 0.5

// --- PID values ---

#define DRIVE_P_VALUE 0.00001
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 0
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 0.000187

#define ROT_P_VALUE 0.4
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0

// --- Swerve module ---

SwerveModule::SwerveModule(int driveCANID, int turningCANID, int canCoderCANID)
  : driveMotor(driveCANID, rev::CANSparkMax::MotorType::kBrushless),
    driveEncoder(driveMotor.GetEncoder()),
    drivePID(driveMotor.GetPIDController()),
    turningMotor(turningCANID, rev::CANSparkMax::MotorType::kBrushless),
    turningRelEncoder(turningMotor.GetEncoder()),
    turningPID(turningMotor.GetPIDController()),
    turningAbsEncoder(canCoderCANID) {
  
    // --- Drive motor config ---
    
    driveMotor.RestoreFactoryDefaults();

    // Set the idle mode to coast (Because the robot will start disabled).
    setIdleMode(COAST);

    // Voltage limit.
    driveMotor.EnableVoltageCompensation(DRIVE_MAX_VOLTAGE);
    // Amperage limit.
    driveMotor.SetSmartCurrentLimit(DRIVE_MAX_AMPERAGE);

    // It is not inverted!
    driveMotor.SetInverted(false);

    // Ramping (0.5 seconds to accelerate from neutral to full throttle).
    driveMotor.SetClosedLoopRampRate(DRIVE_RAMP_TIME);
    driveMotor.SetOpenLoopRampRate(DRIVE_RAMP_TIME);

    // Frame period.
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);

    // Make sure the PID controller uses the encoder for its error correction.
    drivePID.SetFeedbackDevice(driveEncoder);
    // PID Values.
    drivePID.SetP(DRIVE_P_VALUE, 0);
    drivePID.SetI(DRIVE_I_VALUE, 0);
    drivePID.SetD(DRIVE_D_VALUE, 0);
    drivePID.SetIZone(DRIVE_I_ZONE_VALUE, 0);
    drivePID.SetFF(DRIVE_FF_VALUE, 0);
  
    // --- Turning motor config ---
    
    turningMotor.RestoreFactoryDefaults();

    // Coast when idle.
    turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // Voltage limit.
    turningMotor.EnableVoltageCompensation(TURN_MAX_VOLTAGE);
    // Amperage limit.
    turningMotor.SetSmartCurrentLimit(TURN_MAX_AMPERAGE);

    // It is not inverted!
    turningMotor.SetInverted(true);

    // Frame period.
    turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
    turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);

    // Make sure the PID controller uses the encoder for its error correction.
    turningPID.SetFeedbackDevice(turningRelEncoder);
    // PID values.
    turningPID.SetP(ROT_P_VALUE, 0);
    turningPID.SetI(ROT_I_VALUE, 0);
    turningPID.SetD(ROT_D_VALUE, 0);
    turningPID.SetIZone(ROT_I_ZONE_VALUE, 0);
    turningPID.SetFF(ROT_FF_VALUE, 0);
    
    // --- CANCoder config ---
    
    turningAbsEncoder.ConfigFactoryDefault();
    // Set the range of the CANCoder to -180 to +180 instead of 0 to 360.
    turningAbsEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
}

SwerveModule::~SwerveModule() {

}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();
  
    // Optimize the target state by flipping motor directions and adjusting
    // rotations in order to turn the least amount of distance possible.
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    
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
    drivePID.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
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
    
    // Convert the radian value to internal encoder value.
    double output = rotation.value() * TURN_RADIAN_TO_ENCODER_FACTOR;
    
    // Add the current relative rotation.
    output += getRelativeRotation();
    
    // Set PID controller reference.
    turningPID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);
}

frc::SwerveModuleState SwerveModule::getState() {
    // The velocity and rotation of the swerve module.
    return { getDriveVelocity(), getAbsoluteRotation() };
}

void SwerveModule::setOffset(units::radian_t offset) {
    canCoderOffset = offset;
}

void SwerveModule::setIdleMode(IdleMode mode) {
    rev::CANSparkMax::IdleMode idleMode;
    
    switch (mode) {
        case BRAKE:
            idleMode = rev::CANSparkMax::IdleMode::kBrake;
            break;
        case COAST:
            idleMode = rev::CANSparkMax::IdleMode::kCoast;
            break;
    }

    // Set the idle mode of the drive motor.
    driveMotor.SetIdleMode(idleMode);
}

units::meters_per_second_t SwerveModule::getDriveVelocity() {
    // Convert rotations per minute to meters per second.
    double mps = (driveEncoder.GetVelocity() / 60) * DRIVE_ENCODER_TO_METER_FACTOR;
    
    return units::meters_per_second_t(mps);
}

double SwerveModule::getRelativeRotation() {
    return turningRelEncoder.GetPosition();
}

frc::Rotation2d SwerveModule::getAbsoluteRotation() {
    // The angle from the CANCoder.
    units::degree_t angle(turningAbsEncoder.GetAbsolutePosition());

    // Subtract the offset from the angle.
    angle -= canCoderOffset;

    // Subtract 90 degrees from the angle because the kinematics thinks that
    // forward is to the right for some reason.
    angle -= 90_deg;

    return angle;
}

// --- Drivetrain ---

Drive::Drive(Limelight* limelight)
  : limelight(limelight) {

    // Configure the calibration time of the IMU to 4 seconds.
    imu.ConfigCalTime(frc::ADIS16470_IMU::CalibrationTime::_4s);
    // Set the default axis for the IMU's gyro to take (Z for up and down).
    imu.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
    // Zero the IMU.
    resetIMU();
    // Calibrate the IMU.
    calibrateIMU();

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
}

void Drive::resetToMode(MatchMode mode) {
    // Cancel the current command.
    cmdCancel();
    
    if (mode == MODE_DISABLED) {
        // Set the drive motors to coast when disabled so they can try to push
        // swerve drive (not going to happen but ok).
        setIdleMode(SwerveModule::COAST);
    }
    else {
        // Set the drive motors to brake in teleop and autonomous modes because
        // we want the robot to stop when we tell it to stop.
        setIdleMode(SwerveModule::BRAKE);
    }
}

void Drive::sendFeedback() {
    Feedback::sendDouble("drive", "imu angle (degrees)", imu.GetAngle().value());

    std::string modeString = "";
    switch (controlMode) {
        case FIELD_CENTRIC:
            modeString = "field centric";
            break;
        case ROBOT_CENTRIC:
            modeString = "robot centric";
            break;
    }
    Feedback::sendString("drive", "control mode", modeString.c_str());
    
    frc::Pose2d pose = getPose();

    Feedback::sendDouble("drive", "robot x position (meters)", pose.X().value());
    Feedback::sendDouble("drive", "robot y position (meters)", pose.Y().value());
    Feedback::sendDouble("drive", "robot rotation (degrees)", pose.Rotation().Degrees().value());

    Feedback::sendDouble("drive", "wheel 0 rotation (degrees)", swerveModules.at(0)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 1 rotation (degrees)", swerveModules.at(1)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 2 rotation (degrees)", swerveModules.at(2)->getState().angle.Degrees().value());
    Feedback::sendDouble("drive", "wheel 3 rotation (degrees)", swerveModules.at(3)->getState().angle.Degrees().value());
    
    //hi josh

    Feedback::sendDouble("drive", "wheel 0 speed (mps)", swerveModules.at(0)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 1 speed (mps)", swerveModules.at(1)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 2 speed (mps)", swerveModules.at(2)->getState().speed.value());
    Feedback::sendDouble("drive", "wheel 3 speed (mps)", swerveModules.at(3)->getState().speed.value());

    Feedback::sendBoolean("drive", "command is running", cmd.running);
    Feedback::sendBoolean("drive", "command total time", cmd.timer.Get().value());
}

void Drive::process() {
    // hi nevin
    // hi jeff :D
    
    // Continue tracking the position and rotation on the field.
    updateOdometry();

    // Execute the current drive command.
    executeCommand();
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

void Drive::zeroRotation() {
    resetIMU();
    updateOdometry();
}

void Drive::calibrateIMU() {
    imu.Calibrate();
}

void Drive::configMagneticEncoders() {
    // Apply the current rotation of the swerve modules to the offsets.
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        units::radian_t angle(swerveModules.at(i)->getState().angle.Radians());
        
        // Subtract the 90 degree offset.
        angle -= 90_deg;
        
        // Add the angle of the module to the offset that is already applied.
        offsets.at(i) += angle;
    }
    
    // Write the new offsets to the offsets file.
    writeOffsetsFile();

    // Apply the new offsets to the swerve modules.
    applyOffsets();
}

void Drive::manualDrive(double xPct, double yPct, double rotPct) {
    // Take control if drive command is running.
    cmdCancel();

    // Calculate the velocities.
    units::meters_per_second_t xVel    = xPct   * DRIVE_MANUAL_MAX_SPEED;
    units::meters_per_second_t yVel    = yPct   * DRIVE_MANUAL_MAX_SPEED;
    units::radians_per_second_t rotVel = rotPct * DRIVE_MANUAL_MAX_ANGULAR_SPEED;
    
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
        // Stop the drive motor.
        swerveModules.at(i)->setDriveMotor(0_mps);
        // Turn the swerve module to point towards the center of the robot.
        swerveModules.at(i)->setTurningMotor(angle);
    }
}

frc::TrajectoryConfig Drive::getTrajectoryConfig() {
    // Generate a configuration with speed and acceleration limits.
    frc::TrajectoryConfig trajectoryConfig { DRIVE_CMD_MAX_SPEED, DRIVE_CMD_MAX_ACCELERATION };

    // Add a constraint to the configuration to make the trajectory better
    // suited for a swerve drive.
    trajectoryConfig.AddConstraint(frc::SwerveDriveKinematicsConstraint<4>(kinematics, DRIVE_CMD_MAX_SPEED));

    // Stupid class does not like copy constructors for some reason.
    return std::move(trajectoryConfig);
}

bool Drive::cmdRotateToCargo() {
    
    // TODO Implement.
    
    return true;
}

bool Drive::cmdRotateToHub() {
    // Make sure the limelight sees a target.
    if (!limelight->hasTarget()) {
        // TODO Use odometry instead.
        return false;
    }
    
    // Get the horizontal angle from limelight.
    units::radian_t angle(limelight->getAngleHorizontal());

    // Start a rotate command.
    cmdRotate(angle);

    return true;
}

void Drive::cmdRotate(frc::Rotation2d angle) {
    // Create a trajectory to rotate the specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { 0_m, 0_m, angle }, getTrajectoryConfig());

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle) {
    // Create a trajectory that drives the specified distance and turns the
    // specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { x, y, angle }, getTrajectoryConfig());

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdFollowPathweaverTrajectory(std::string jsonPath) {
    fs::path directory = fs::path(frc::filesystem::GetDeployDirectory()) / jsonPath;
    
    // Load the trajectory from the Pathweaver json file.
    frc::Trajectory trajectory(frc::TrajectoryUtil::FromPathweaverJson(directory.string()));

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdFollowTrajectory(frc::Trajectory trajectory) {
    cmd.trajectory = trajectory;
    cmd.timer.Reset();
    cmd.timer.Start();
    cmd.running = true;
}

bool Drive::cmdIsFinished() {
    // If a command is currently running but the total time of the trajectory
    // has elapsed.
    if (cmd.running && cmd.timer.HasElapsed(cmd.trajectory.TotalTime())) {
        // Stop the command.
        cmdCancel();
        // Stop all the swerve modules.
        setModuleStates({0_mps, 0_mps, 0_rad_per_s});
    }
    return !cmd.running;
}

void Drive::cmdCancel() {
    cmd.timer.Stop();
    cmd.running = false;
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
    // Generate module states using the chassis velocities. This is the magic
    // function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
    
    // Recalculate the wheel velocities relative to the max speed.
    kinematics.DesaturateWheelSpeeds(&moduleStates, DRIVE_MANUAL_MAX_SPEED);
    
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
    imu.Reset();
}

frc::Rotation2d Drive::getRotation() {
    // Get the current rotation of the robot.
    units::radian_t rotation(units::math::fmod(imu.GetAngle(), 360_deg));

    // Convert -2π to 2π value into -π to π value.
    if(units::math::abs(rotation).value() > wpi::numbers::pi) {
        rotation = units::radian_t(rotation.value() - (2 * wpi::numbers::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    return frc::Rotation2d(rotation);
}

void Drive::executeCommand() {
    // Only execute a command if the command is running.
    if (cmdIsFinished()) {
        return;
    }

    // Get the current time of the trajectory.
    units::second_t currentTime(cmd.timer.Get());
    
    // Sample the desired state of the trajectory at this point in time.
    frc::Trajectory::State desiredState = cmd.trajectory.Sample(currentTime);
    
    // Calculate chassis velocities that are required in order to reach the
    // desired state.
    frc::ChassisSpeeds targetChassisSpeeds = cmdController.Calculate(
        getPose(), desiredState, cmd.trajectory.States().back().pose.Rotation());

    // Finally drive!
    setModuleStates(targetChassisSpeeds);

    //hi ishan
    //hi jeff
    //hi trevor
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
        swerveModules.at(i)->setOffset(offsets.at(i));
    }
}

void Drive::setIdleMode(SwerveModule::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}