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
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <frc/ADIS16470_IMU.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <wpi/array.h>
#include <wpi/numbers>
#include <wpi/fs.h>
#include <optional>

#define DRIVE_MAX_SPEED 4_mps
#define AUTO_MAX_SPEED 1_mps
#define AUTO_MAX_ACCELERATION 0.4_mps_sq
#define AUTO_TRAJECTORY_CONFIG (frc::TrajectoryConfig(AUTO_MAX_SPEED, AUTO_MAX_ACCELERATION))

#define ROBOT_WIDTH 0.54_m
#define ROBOT_LENGTH 0.67_m

// --- Swerve module ---

/**
 * Represents a singular swerve module on the robot.
 */
class SwerveModule {
public:
    SwerveModule(int driveCANID, int turningCANID, int canCoderCANID, double canCoderOffset);
    ~SwerveModule();

    /**
     * Sets the state of the swerve module (Velocity and angle).
     */
    void setState(frc::SwerveModuleState state);

    /**
     * Returns the current state of the swerve module.
     */
    frc::SwerveModuleState getState();

private:
    /**
     * Sets the speed of the drive motor (-1 to 1).
     */
    void setDriveMotor(double speed);

    /**
     * Returns the current velocity (RPM) of the drive motor.
     */
    double getDriveVelocity();

    /**
     * Sets the angle of the swerve module.
     */
    void setTurningMotor(units::radian_t angle);

    /**
     * Returns the relative rotation of the module (NEO 550 internal encoder).
     */
    double getRelativeRotation();

    /**
     * Returns the absolute rotation of the module (CANCoder).
     */
    units::radian_t getAbsoluteRotation();

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

    // The offset of the CANCoder.
    const units::radian_t canCoderOffset;
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
     * Returns the current pose (X, Y position and rotation).
     */
    frc::Pose2d getPose();

    /**
     * Resets the forward direction of the robot during field-centric control.
     */
    void zeroRotation();

    /**
     * Calibrates the IMU (Should be done only once at a time when the robot is
     * not moving).
     */
    void calibrateIMU();
    
    /**
     * Manually set the velocities of the robot (dependant on control type).
     */
    void manualDrive(double xVel, double yVel, double rotVel);

    enum ControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    };

    /**
     * Sets the control mode of the robot (relative to the robot or relative to
     * the field).
     */
    void setControlMode(ControlMode mode);

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
     * Begins a command to follow a specified pathweaver trajectory.
     */
    void cmdFollowPathWeaverTrajectory(const char* pathweaverJson);

    /**
     * Begins a command to follow a specified trajectory.
     */
    void cmdFollowTrajectory(frc::Trajectory trajectory);

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
     * Generates and sends states to the swerve modules from chassis speeds.
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
     * Returns the rotation of the robot.
     */
    frc::Rotation2d getRotation();

    /**
     * Executes the current command.
     */
    void executeCommand();

    // The control mode of the robot.
    ControlMode controlMode = FIELD_CENTRIC;

    // The limelight sensor.
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
      new SwerveModule(CAN_SWERVE_FL_DRIVE_MOTOR, CAN_SWERVE_FL_ROT_MOTOR, CAN_SWERVE_FL_ROT_CAN_CODER, +0),
      new SwerveModule(CAN_SWERVE_BL_DRIVE_MOTOR, CAN_SWERVE_BL_ROT_MOTOR, CAN_SWERVE_BL_ROT_CAN_CODER, +0),
      new SwerveModule(CAN_SWERVE_BR_DRIVE_MOTOR, CAN_SWERVE_BR_ROT_MOTOR, CAN_SWERVE_BR_ROT_CAN_CODER, +0),
      new SwerveModule(CAN_SWERVE_FR_DRIVE_MOTOR, CAN_SWERVE_FR_ROT_MOTOR, CAN_SWERVE_FR_ROT_CAN_CODER, +0),
    };

    // The helper class that converts chassis speeds into swerve module states.
    frc::SwerveDriveKinematics<4> kinematics { locations };

    // The odometry class that tracks the position of the robot on the field.
    frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };

    // The ADIS16470 IMU (3D gyro and accelerometer) in the SPI port on the
    // roborio.
    frc::ADIS16470_IMU imu {};

    struct SwerveCommand {
        frc::Trajectory trajectory;
        frc::Timer timer;
        bool running = false;
    };

    SwerveCommand cmd = {};

    frc::HolonomicDriveController cmdController { { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0, {} } };
};