#if 0
#pragma once
#include "rev/CANSparkMax.h"

class ThunderSparkMaxImpl;

class ThunderSparkMaxCANPIDController {
public:
    ThunderSparkMaxCANPIDController();

    virtual rev::CANError SetOutputRange(double min, double max, int slotID = 0);
    virtual rev::CANError SetFF(double gain, int slotID = 0);
    virtual rev::CANError SetP(double gain, int slotID = 0);
    virtual rev::CANError SetI(double gain, int slotID = 0);
    virtual rev::CANError SetD(double gain, int slotID = 0);
    virtual rev::CANError SetIZone(double IZone, int slotID = 0);
    virtual rev::CANError SetReference(double value, rev::ControlType ctrl, int pidSlot = 0,
            double arbFeedforward = 0, 
            rev::CANPIDController::ArbFFUnits arbFFUnits = rev::CANPIDController::ArbFFUnits::kVoltage);
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
        // Rate is amount of time to go from 0 to full throttle in seconds
        virtual void SetOpenLoopRampRate(double rate) = 0;
        
        // Returns rotations of encoder
        virtual double GetEncoder() = 0;
        virtual void SetEncoder(double rotations) = 0;

        virtual double GetVelocity() = 0;

        enum IdleMode {BRAKE, COAST};
        virtual void SetIdleMode(IdleMode idleMode) = 0;
        virtual void SetInverted(bool inverted) = 0;
        virtual void Follow(ThunderSparkMax *leader, bool invertOutput = false) = 0;

        virtual ThunderSparkMaxCANPIDController *getSparkPID() = 0;
        virtual double GetOutputCurrent() = 0;

        enum MotorID {
            // NOTE: Implementation depends on the order of these!!
            // Do not reorder/add/remove anything from this without
            // also updating the implementation to match!!
            DriveFrontLeft = 0,
            DriveFrontRight,
            DriveRearLeft,
            DriveRearRight,
            IntakePivot,
            IntakeBeaterBars,
            StorageAgitator,
            StorageTransitionToShooter,
            ShooterPrimer, 
            ShooterLeft,
            ShooterRight,
            WheelOfFortune,
            Hang,
        };
        static ThunderSparkMax *create(MotorID id);
};

#endif