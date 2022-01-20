#pragma once

#include "IOMap.h"
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/ADIS16470_IMU.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <wpi/array.h>
#include <wpi/numbers>

#define MAX_SPEED 4_mps
#define AUTO_SPEED 1_mps

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
     * Sets the state of the swerve module.
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
 * Represents the drivetrain of the robot and handles all drive-related functionality.
 */
class Drive {
public:
    Drive();
    ~Drive();

    void process();
    
    /**
     * Returns the rotation of the robot.
     */
    frc::Rotation2d getRotation();

    /**
     * Resets the rotation of the robot (field-centric).
     */
    void resetForward();

    /**
     * Calibrates the IMU.
     */
    void calibrateIMU();

    /**
     * Sets the velocities of the robot.
     */
    void setVelocities(double xVel, double yVel, double rotVel, bool fieldCentric = true);
    
    /**
     * Begins a command to rotate a specified angle.
     */
    void cmdRotate(units::radian_t anlge);

    /**
     * Begins a command to drive a specified distance.
     */
    void cmdDrive(units::meter_t x, units::meter_t y, units::radian_t angle = units::radian_t(0));
    
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
     * Returns the current pose.
     */
    frc::Pose2d getPose();

    /**
     * Resets the IMU to 0.
     */
    void resetIMU();

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
    
    // The ADIS16470 IMU (3D gyro and accelerometer).

    frc::ADIS16470_IMU imu {};
};