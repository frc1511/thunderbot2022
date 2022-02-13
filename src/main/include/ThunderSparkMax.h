#pragma once
#include "rev/CANSparkMax.h"

class ThunderSparkMaxImpl;

class ThunderSparkMaxCANPIDController {
public:
    ThunderSparkMaxCANPIDController();

    virtual rev::REVLibError SetOutputRange(double min, double max, int slotID = 0);
    virtual rev::REVLibError SetFF(double gain, int slotID = 0);
    virtual rev::REVLibError SetP(double gain, int slotID = 0);
    virtual rev::REVLibError SetI(double gain, int slotID = 0);
    virtual rev::REVLibError SetD(double gain, int slotID = 0);
    virtual rev::REVLibError SetIZone(double IZone, int slotID = 0);
    virtual rev::REVLibError SetReference(double value, rev::CANSparkMax::ControlType ctrl, int pidSlot = 0,
            double arbFeedforward = 0, 
            rev::SparkMaxPIDController::ArbFFUnits arbFFUnits = rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
};

/**
 * Wrapping of SparkMax providing the same API but having several
 * implementations backed by not only spark max but also TalonSRX and legacy
 * motor controllers. Allows for flexibility in test setups whilst retaining
 * common API.
 */
class ThunderSparkMax {
    public:
        ThunderSparkMax();
        virtual ~ThunderSparkMax();

        virtual void Set(double speed) = 0;
        virtual double Get() = 0;

        // Returns rotations of encoder
        virtual double GetEncoder() = 0;
        virtual void SetEncoder(double rotations) = 0;

        virtual double GetVelocity() = 0;
        virtual double GetOutputCurrent() = 0;
        virtual ThunderSparkMaxCANPIDController *GetPIDController() = 0;

        /*********************************************/
        // Configuration methods

        enum IdleMode { BRAKE, COAST };
        virtual void SetIdleMode(IdleMode idleMode) = 0;
        virtual void SetInverted(bool inverted) = 0;
        // Rate is amount of time to go from 0 to full throttle in seconds
        virtual void SetOpenLoopRampRate(double rate) = 0;
        virtual void SetClosedLoopRampRate(double rate) = 0;
        virtual void Follow(ThunderSparkMax *leader, bool invertOutput = false) = 0;
        virtual void SetSmartCurrentLimit(unsigned int limitAmps) = 0;
        virtual void EnableVoltageCompensation(double nominalVoltage) = 0;

        virtual void RestoreFactoryDefaults() = 0;
        virtual void BurnFlash() = 0;



        /*********************************************/
        // Creation factory
        /*********************************************/

        enum MotorID {
            // NOTE: Implementation depends on the order of these!!
            // Do not reorder/add/remove anything from this without
            // also updating the implementation to match!!
            DriveFrontLeft = 0,
            DriveFrontRight,
            DriveRearLeft,
            DriveRearRight,
            DrivePivotFrontLeft,
            DrivePivotFrontRight,
            DrivePivotRearLeft,
            DrivePivotRearRight,
            StorageStage1,
            StorageStage2,
            ShooterLeft,
            ShooterRight,
            Hang,
        };
        
        static ThunderSparkMax *create(MotorID id);
};
