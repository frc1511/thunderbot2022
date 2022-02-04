#pragma once

#include "Mechanism.h"
#include "IOMap.h"
#include "Limelight.h"
#include "Feedback.h"
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <wpi/array.h>
#include <wpi/numbers>
#include <wpi/fs.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

// The maximum speed of the chassis during manual drive.
#define DRIVE_MANUAL_MAX_SPEED 4_mps
// The maximum angular speed of the chassis during manual drive.
#define DRIVE_MANUAL_MAX_ANGULAR_SPEED 3.14_rad_per_s

// The maximum speed of the chassis during a drive command.
#define DRIVE_CMD_MAX_SPEED 1_mps
// The maximum acceleration of the chassis during a drive command.
#define DRIVE_CMD_MAX_ACCELERATION 0.4_mps_sq
// The maximum angular speed of the chassis during a drive command.
#define DRIVE_CMD_MAX_ANGULAR_SPEED 3.14_rad_per_s
// The maximum angular acceleration of the chassis during a drive command.
#define DRIVE_CMD_MAX_ANGULAR_ACCELERATION (3.14_rad_per_s / 1_s)

// The width of the robot.
#define ROBOT_WIDTH 0.54_m
// The length of the robot.
#define ROBOT_LENGTH 0.67_m

// --- Swerve module ---

/**
 * Represents a singular swerve module on the robot.
 */
class SwerveModule {
public:
    SwerveModule(int driveCANID, int turningCANID, int canCoderCANID);
    ~SwerveModule();

    /**
     * Sets the state of the swerve module (Velocity and angle).
     */
    void setState(frc::SwerveModuleState state);

    /**
     * Set the velocity of the drive motor (meters per second).
     */
    void setDriveMotor(units::meters_per_second_t velocity);

    /**
     * Sets the angle of the swerve module.
     */
    void setTurningMotor(units::radian_t angle);

    /**
     * Returns the current state of the swerve module (Velocity and angle).
     */
    frc::SwerveModuleState getState();

    /**
     * Applies an offset to the CANCoder.
     */
    void setOffset(units::radian_t offset);

    enum IdleMode {
        BRAKE,
        COAST,
    };

    /**
     * Sets the idle mode of the module.
     */
    void setIdleMode(IdleMode mode);

private:
    /**
     * Returns the current velocity of the drive motor (meters per second).
     */
    units::meters_per_second_t getDriveVelocity();

    /**
     * Returns the relative rotation of the module (Rotations of the NEO 550).
     */
    double getRelativeRotation();

    /**
     * Returns the absolute rotation of the module (CANCoder encoder value).
     */
    frc::Rotation2d getAbsoluteRotation();

    // The drive motor (NEO Brushless motor).
    rev::CANSparkMax driveMotor;
    rev::SparkMaxRelativeEncoder driveEncoder;
    rev::SparkMaxPIDController drivePID;

    // The turning motor (NEO 550).
    rev::CANSparkMax turningMotor;
    rev::SparkMaxRelativeEncoder turningRelEncoder;
    rev::SparkMaxPIDController turningPID;

    // The absolute encoder (CTRE CANCoder).
    ctre::phoenix::sensors::CANCoder turningAbsEncoder;

    // The offset of the CANCoders.
    units::radian_t canCoderOffset = 0_rad;
};

// --- Drivetrain ---

/**
 * Represents the drivetrain of the robot and handles all drive-related
 * functionality.
 */
class Drive : public Mechanism {
public:
    Drive(Limelight* limelight);
    ~Drive();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;
        
    /**
     * Returns the current pose of the odometry (X, Y position and rotation
     * tracked by odometry).
     */
    frc::Pose2d getPose();

    /**
     * Resets the forward direction of the robot during field-centric control.
     */
    void zeroRotation();

    /**
     * Calibrates the IMU for 4 seconds (Should only be done once at the
     * beginning of the match when the robot is not moving).
     */
    void calibrateIMU();

    /**
     * Applies the current rotation of the swerve modules as the offset of the
     * magnetic encoders. *** IMPORTANT *** Should only be called after
     * replacing a swerve module and when all the swerve modules are rotated
     * towards the front of the robot!
     */
    void configMagneticEncoders();
    
    /**
     * Manually control the robot using percentages of the max drive velocities.
     * (The direction of the velocities is dependant on the control type).
     */
    void manualDrive(double xPct, double yPct, double rotPct);

    enum ControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    };

    /**
     * Sets the control mode of the robot (relative to the robot or relative to
     * the field).
     */
    void setControlMode(ControlMode mode);

    /**
     * Rotates all the swerve modules towards the center of the robot in order
     * to reduce pushing by other robots (aka making the robot into a brick).
     */
    void makeBrick();

    /**
     * Returns the configuration for all trajectories.
     */
    frc::TrajectoryConfig getTrajectoryConfig();

    // --- Commands ---

    /**
     * Commands are used primarily during the autonomous period in order to
     * instruct the drivetrain to execute an action, such as rotating, driving,
     * or following a trajectory. Commands are mutually exclusive, meaning that
     * when the drive is instructed to execute a command, it will abort any
     * other ongoing commands and immediately begin the new one.
     */

    /**
     * Begins a command to rotate the robot towards the cargo using the forward-
     * facing camera. Returns whether it has successfully identified a cargo.
     */
    bool cmdRotateToCargo();

    /**
     * Begins a command to rotate the robot to the high hub using limelight.
     * Returns whether it has successfully identified the high hub.
     */
    bool cmdRotateToHub();

    /**
     * Begins a command to rotate a specified angle.
     */
    void cmdRotate(frc::Rotation2d angle);

    /**
     * Begins a command to drive a specified distance and rotate a specified
     * angle.
     */
    void cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle = frc::Rotation2d());
    
    /**
     * Begins a command to follow a specified trajectory.
     */
    void cmdFollowTrajectory(frc::Trajectory trajectory);

    /**
     * Begins a command to follow a pathweaver trajecectory from a json file.
     */
    void cmdFollowPathweaverTrajectory(std::string jsonPath);

    /**
     * Returns whether the last command has finished.
     */
    bool cmdIsFinished();

    /**
     * Cancels the current command.
     */
    void cmdCancel();

private:
    /**
     * Generates and sends states to the swerve modules using specified chassis
     * speeds.
     */
    void setModuleStates(frc::ChassisSpeeds chassisSpeeds);

    /**
     * Updates the position and rotation on the field.
     */
    void updateOdometry();

    /**
     * Resets the position and rotation on the field.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Resets the IMU to 0.
     */
    void resetIMU();
    
    /**
     * Returns the rotation of the robot on the field.
     */
    frc::Rotation2d getRotation();

    /**
     * Executes the current command.
     */
    void executeCommand();

    /**
     * Reads the magnetic encoder offsets file.
     */
    bool readOffsetsFile();

    /**
     * Writes the current magnetic encoder offsets into the file.
     */
    void writeOffsetsFile();

    /**
     * Applies the current magnetic encoder offsets to the swerve modules.
     */
    void applyOffsets();

    /**
     * Sets the idle mode of the drive motors.
     */
    void setIdleMode(SwerveModule::IdleMode mode);

    // The control mode of the robot.
    ControlMode controlMode = FIELD_CENTRIC;

    // The limelight vision sensor.
    Limelight* limelight;

    // The locations of the swerve modules on the robot.
    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
        frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
    };

    // The swerve modules on the robot.
    wpi::array<SwerveModule*, 4> swerveModules {
      new SwerveModule(CAN_SWERVE_FL_DRIVE_MOTOR, CAN_SWERVE_FL_ROT_MOTOR, CAN_SWERVE_FL_ROT_CAN_CODER),
      new SwerveModule(CAN_SWERVE_BL_DRIVE_MOTOR, CAN_SWERVE_BL_ROT_MOTOR, CAN_SWERVE_BL_ROT_CAN_CODER),
      new SwerveModule(CAN_SWERVE_BR_DRIVE_MOTOR, CAN_SWERVE_BR_ROT_MOTOR, CAN_SWERVE_BR_ROT_CAN_CODER),
      new SwerveModule(CAN_SWERVE_FR_DRIVE_MOTOR, CAN_SWERVE_FR_ROT_MOTOR, CAN_SWERVE_FR_ROT_CAN_CODER),
    };

    // The magnetic encoder offsets of the swerve modules.
    wpi::array<units::radian_t, 4> offsets { 0_rad, 0_rad, 0_rad, 0_rad };

    /**
     * The helper class that can be used to convert chassis speeds into swerve
     * module states.
     */
    frc::SwerveDriveKinematics<4> kinematics { locations };

    /**
     * The class that will handle tracking the position of the robot on the
     * field during the match.
     */
    frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };

    /**
     * Our super accurate IMU (3d gyro and accelerometer) in the SPI port on the
     * roborio.
     */
    frc::ADIS16470_IMU imu {};

    /**
     * Represents a command for the drivetrain to execute over a period of time.
     */
    struct SwerveCommand {
        frc::Trajectory trajectory;
        frc::Timer timer;
        bool running = false;
    };

    // The current command.
    SwerveCommand cmd {};

    /**
     * The class that will handle generating chassis speeds for the robot using
     * trajectory states. Implements a PID-style error correction system to
     * accurately drive to a position.
     */
    frc::HolonomicDriveController cmdController {
        // PID Values for movement in the X direction.
        { 1, 0, 0 },
        // PID Values for movement in the Y direction.
        { 1, 0, 0 },
        // PID Values for rotational movement, and rotational constraints profile.
        { 1, 0, 0, frc::TrapezoidProfile<units::radians>::Constraints(DRIVE_CMD_MAX_ANGULAR_SPEED, DRIVE_CMD_MAX_ANGULAR_ACCELERATION) }
    };
};