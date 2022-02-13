#include "ThunderSparkMax.h"
#include "IOMap.h"

class ThunderSparkMaxImpl : public ThunderSparkMax {
    public:
        ThunderSparkMaxImpl(const char *implName, int id);
        virtual rev::CANSparkMax *getSparkMax();
        virtual void Set(double speed);
        virtual double Get();
        
        virtual void SetOpenLoopRampRate(double rate);

        // Returns rotations of encoder
        virtual double GetEncoder() ;
        virtual void SetEncoder(double rotations);

        virtual double GetVelocity();

        virtual void SetIdleMode(IdleMode idleMode);
        virtual void SetInverted(bool inverted);
        virtual void Follow(ThunderSparkMax *leader, bool invertOutput = false);

        virtual ThunderSparkMaxCANPIDController* GetPIDController();
        virtual double GetOutputCurrent();

        virtual void SetSmartCurrentLimit(unsigned int limitAmps);
        virtual void EnableVoltageCompensation(double nominalVoltage);

        virtual void RestoreFactoryDefaults();
        virtual void BurnFlash();


    protected:
        const char *implName;
        int id;

        void printWarning(bool *hasPrinted, const char *description);
        double lastSpeed;
        bool printedPIDWarning;
        bool printedEncoderWarning;
        bool printedIdleWarning;
        bool printedInvertWarning;
        bool printedFollowWarning;
        bool printedRampWarning;
        bool printedCurrentWarning;
        bool printedVelocityWarning;
        bool printedVcompWarning;
        bool printedCurrentLimitWarning;
        bool printedFlashWarning;

        static ThunderSparkMaxCANPIDController fakePIDController;
};

ThunderSparkMaxImpl::ThunderSparkMaxImpl(const char *implName, int id) :
        implName(implName),
        id(id),
        lastSpeed(0),
        printedPIDWarning(false),
        printedEncoderWarning(false),
        printedIdleWarning(false),
        printedInvertWarning(false),
        printedFollowWarning(false),
        printedRampWarning(false),
        printedCurrentWarning(false),
        printedVelocityWarning(false),
        printedVcompWarning(false),
        printedCurrentLimitWarning(false),
        printedFlashWarning(false)
{
    
}

ThunderSparkMaxCANPIDController ThunderSparkMaxImpl::fakePIDController;


rev::CANSparkMax *ThunderSparkMaxImpl::getSparkMax()
{
    return NULL;
}

void ThunderSparkMaxImpl::Set(double speed)
{
    lastSpeed = speed;
}

double ThunderSparkMaxImpl::Get()
{
    return lastSpeed;
}

void ThunderSparkMaxImpl::SetOpenLoopRampRate(double rate)
{
    printWarning(&printedRampWarning, "Output Ramping");
}
        
double ThunderSparkMaxImpl::GetEncoder()
{
    printWarning(&printedEncoderWarning, "Encoder");
    return 0;
}

void ThunderSparkMaxImpl::SetEncoder(double rotations)
{
    printWarning(&printedEncoderWarning, "Encoder");
}

double ThunderSparkMaxImpl::GetVelocity()
{
    printWarning(&printedVelocityWarning, "Getting Velocity");
    return 0;
}

void ThunderSparkMaxImpl::SetIdleMode(IdleMode idleMode)
{
    printWarning(&printedIdleWarning, "Idle Mode");
}

void ThunderSparkMaxImpl::SetInverted(bool inverted)
{
    printWarning(&printedInvertWarning, "Invert of output");
}
void ThunderSparkMaxImpl::Follow(ThunderSparkMax *leader, bool invertOutput)
{
    printWarning(&printedFollowWarning, "Follower Mode");
}

ThunderSparkMaxCANPIDController* ThunderSparkMaxImpl::GetPIDController() {
    printWarning(&printedPIDWarning, "Native PID");
    return &fakePIDController;
}

double ThunderSparkMaxImpl::GetOutputCurrent()
{
    printWarning(&printedCurrentWarning, "Current");
    return 0;
}

void ThunderSparkMaxImpl::SetSmartCurrentLimit(unsigned int limitAmps)
{
    printWarning(&printedCurrentLimitWarning, "CurrentLimit");
}

void ThunderSparkMaxImpl::EnableVoltageCompensation(double nominalVoltage)
{
    printWarning(&printedVcompWarning, "VoltageCompensation");
}

void ThunderSparkMaxImpl::BurnFlash()
{
    printWarning(&printedFlashWarning, "RestoreFactoryDefaults/BurnFlash");
}

void ThunderSparkMaxImpl::RestoreFactoryDefaults()
{
    printWarning(&printedFlashWarning, "RestoreFactoryDefaults/BurnFlash");
}

void ThunderSparkMaxImpl::printWarning(bool *hasPrinted, const char *description)
{
    if (!(*hasPrinted)) {
        printf("WARNING: %s %d does not support %s!\n", implName, id, description);
        *hasPrinted = true;
    }
}


class ThunderSparkMaxCANPIDControllerImpl : public ThunderSparkMaxCANPIDController
{
public:
    ThunderSparkMaxCANPIDControllerImpl(rev::SparkMaxPIDController *pid);

    virtual rev::REVLibError SetOutputRange(double min, double max, int slotID = 0);
    virtual rev::REVLibError SetFF(double gain, int slotID = 0);
    virtual rev::REVLibError SetP(double gain, int slotID = 0);
    virtual rev::REVLibError SetI(double gain, int slotID = 0);
    virtual rev::REVLibError SetD(double gain, int slotID = 0);
    virtual rev::REVLibError SetIZone(double IZone, int slotID = 0);
    virtual rev::REVLibError SetReference(double value, rev::CANSparkMax::ControlType ctrl, int pidSlot = 0,
            double arbFeedforward = 0, 
            rev::SparkMaxPIDController::ArbFFUnits arbFFUnits = rev::SparkMaxPIDController::ArbFFUnits::kVoltage);

private:
    rev::SparkMaxPIDController *pid;
};






#include <frc/motorcontrol/Jaguar.h>
class ThunderSMJagImpl : public ThunderSparkMaxImpl {
    public:
        ThunderSMJagImpl(int pwmChannel);
        virtual void Set(double speed);

    private:
        frc::Jaguar jaguar;
};

#include <ctre/Phoenix.h>
class ThunderSMTalonImpl : public ThunderSparkMaxImpl {
    public:
        ThunderSMTalonImpl(int canId);
        virtual void Set(double speed);

        virtual void SetOpenLoopRampRate(double rate);

        virtual double GetEncoder();
        virtual void SetEncoder(double rotations);
        virtual void SetInverted(bool invert);

        virtual void SetIdleMode(IdleMode idleMode);
    private:
        TalonSRX talon;
};

class ThunderSMCANImpl : public ThunderSparkMaxImpl {
    public:
        ThunderSMCANImpl(int canId);
        virtual void Set(double speed);
        
        virtual void SetOpenLoopRampRate(double rate);

        virtual double GetEncoder();
        virtual double Get();
        virtual void SetEncoder(double rotations);
        virtual void SetInverted(bool invert);

        virtual void SetIdleMode(IdleMode idleMode);
        virtual void Follow(ThunderSparkMax *leader, bool invertOutput = false);

        virtual rev::CANSparkMax *getSparkMax() { return &spark; }
        virtual ThunderSparkMaxCANPIDController *GetPIDController() { return &pid; }
        virtual double GetVelocity() { return enc.GetVelocity(); }

        virtual double GetOutputCurrent() { return spark.GetOutputCurrent();}
        virtual void SetSmartCurrentLimit(unsigned int limitAmps) { spark.SetSmartCurrentLimit(limitAmps); }
        virtual void EnableVoltageCompensation(double nominalVoltage) { spark.EnableVoltageCompensation(nominalVoltage); }

        virtual void BurnFlash() { spark.BurnFlash(); }
        virtual void RestoreFactoryDefaults() { spark.RestoreFactoryDefaults(); }

    private:
        rev::CANSparkMax spark;
        rev::SparkMaxRelativeEncoder enc;
        rev::SparkMaxPIDController smpid;
        ThunderSparkMaxCANPIDControllerImpl pid;
};

#include <frc/motorcontrol/PWMSparkMax.h>
class ThunderSMPWMImpl : public ThunderSparkMaxImpl {
    public:
        ThunderSMPWMImpl(int canId);
        virtual void Set(double speed);
    private:
        frc::PWMSparkMax spark;
};


class ThunderSMFakeImpl : public ThunderSparkMaxImpl {
    public:
        ThunderSMFakeImpl();
        virtual void Set(double speed);
};


struct MotorConfig {
    enum ControllerType {
        Jaguar,
        Talon,
        PWMSparkMax,
        CANSparkMax,
        Fake
    } controllerType;

    int channelOrId;

    MotorConfig(ControllerType controllerType, int channelOrId) : controllerType(controllerType), channelOrId(channelOrId) {}
};


ThunderSparkMaxCANPIDController::ThunderSparkMaxCANPIDController()
{
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetOutputRange(double min, double max, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetFF(double gain, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetP(double gain, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetI(double gain, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetD(double gain, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetIZone(double IZone, int slotID)
{
    return rev::REVLibError::kOk;
}

rev::REVLibError ThunderSparkMaxCANPIDController::SetReference(double value, rev::CANSparkMax::ControlType ctrl, int pidSlot,
        double arbFeedforward, 
        rev::SparkMaxPIDController::ArbFFUnits arbFFUnits)
{
    return rev::REVLibError::kOk;
}

ThunderSparkMaxCANPIDControllerImpl::ThunderSparkMaxCANPIDControllerImpl(rev::SparkMaxPIDController *pid) : pid(pid)
{
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetOutputRange(double min, double max, int slotID)
{
    return pid->SetOutputRange(min, max, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetFF(double gain, int slotID)
{
    return pid->SetFF(gain, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetP(double gain, int slotID)
{
    return pid->SetP(gain, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetI(double gain, int slotID)
{
    return pid->SetI(gain, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetD(double gain, int slotID)
{
    return pid->SetD(gain, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetIZone(double IZone, int slotID)
{
    return pid->SetIZone(IZone, slotID);
}

rev::REVLibError ThunderSparkMaxCANPIDControllerImpl::SetReference(double value, rev::CANSparkMax::ControlType ctrl, int pidSlot,
        double arbFeedforward, 
        rev::SparkMaxPIDController::ArbFFUnits arbFFUnits)
{
    return pid->SetReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
}



ThunderSparkMax::ThunderSparkMax()
{
}

ThunderSparkMax::~ThunderSparkMax()
{
}

ThunderSparkMax *ThunderSparkMax::create(MotorID id)
{
    static MotorConfig config[] = {
#ifndef TEST_BOARD
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_FL_DRIVE_MOTOR),  // DriveFrontLeft
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_FR_DRIVE_MOTOR),  // DriveFrontRight,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_BL_DRIVE_MOTOR),  // DriveRearLeft,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_BR_DRIVE_MOTOR),  // DriveRearRight,

        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_FL_ROT_MOTOR),    // DrivePivotFrontLeft
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_FR_ROT_MOTOR),    // DrivePivotFrontRight,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_BL_ROT_MOTOR),    // DrivePivotRearLeft,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SWERVE_BR_ROT_MOTOR),    // DrivePivotRearRight,

        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_STORAGE_STAGE_ONE),      // StorageStage1,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_STORAGE_STAGE_TWO),      // StorageStage2,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SHOOTER_LEFT_FLYWHEEL_MOTOR),  // ShooterLeft,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SHOOTER_RIGHT_FLYWHEEL_MOTOR), // ShooterRight,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_HANG_WINCH_MOTOR),       // Hang,
#else
        MotorConfig(MotorConfig::ControllerType::Fake, 0),    // DriveFrontLeft
        MotorConfig(MotorConfig::ControllerType::Fake, 1),    // DriveFrontRight,
        MotorConfig(MotorConfig::ControllerType::Fake, 2),    // DriveRearLeft,
        MotorConfig(MotorConfig::ControllerType::Fake, 3),    // DriveRearRight,

        MotorConfig(MotorConfig::ControllerType::Fake, 0),    // DrivePivotFrontLeft
        MotorConfig(MotorConfig::ControllerType::Fake, 1),    // DrivePivotFrontRight,
        MotorConfig(MotorConfig::ControllerType::Fake, 2),    // DrivePivotRearLeft,
        MotorConfig(MotorConfig::ControllerType::Fake, 3),    // DrivePivotRearRight,

        MotorConfig(MotorConfig::ControllerType::Talon, 2),   // StorageStage1,
        MotorConfig(MotorConfig::ControllerType::Talon, 8),   // StorageStage2,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, 9),    // ShooterLeft,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, 10),   // ShooterRight,
        MotorConfig(MotorConfig::ControllerType::Jaguar, 2),    // Hang,
#endif
    };

    ThunderSparkMax *c = NULL;

    switch (config[id].controllerType)
    {
        case MotorConfig::ControllerType::PWMSparkMax:
            c = new ThunderSMPWMImpl(config[id].channelOrId);
            break;
        case MotorConfig::ControllerType::CANSparkMax:
            c = new ThunderSMCANImpl(config[id].channelOrId);
            break;
        case MotorConfig::ControllerType::Talon:
            c = new ThunderSMTalonImpl(config[id].channelOrId);
            break;
        case MotorConfig::ControllerType::Jaguar:
            c = new ThunderSMJagImpl(config[id].channelOrId);
            break;
        case MotorConfig::ControllerType::Fake:
            c = new ThunderSMFakeImpl();
            break;
    }

    return c;
}



/**************************************************************/
// ThunderSMJagImpl

ThunderSMJagImpl::ThunderSMJagImpl(int pwmChannel) : ThunderSparkMaxImpl("Jaguar", pwmChannel), jaguar(pwmChannel)
{
}

void ThunderSMJagImpl::Set(double speed)
{
    jaguar.Set(speed);
    ThunderSparkMaxImpl::Set(speed);
}



/**************************************************************/
// ThunderSMTalonImpl

ThunderSMTalonImpl::ThunderSMTalonImpl(int canid) : ThunderSparkMaxImpl("CAN Talon", canid), talon(canid)
{
    talon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
}

void ThunderSMTalonImpl::Set(double speed)
{
    talon.Set(ControlMode::PercentOutput, speed);
    ThunderSparkMaxImpl::Set(speed);
}

void ThunderSMTalonImpl::SetOpenLoopRampRate(double rate)
{
    talon.ConfigOpenloopRamp(rate);
}

double ThunderSMTalonImpl::GetEncoder()
{
    int n = talon.GetSelectedSensorPosition(0);
    double revs = ((double)n) / 4096;
    return revs;
}

void ThunderSMTalonImpl::SetEncoder(double rotations)
{
    int n = (int)(rotations * 4096);
    talon.SetSelectedSensorPosition(n, 0, 0);
}

void ThunderSMTalonImpl::SetIdleMode(IdleMode idleMode)
{
    talon.SetNeutralMode(idleMode == IdleMode::BRAKE ? NeutralMode::Brake : NeutralMode::Coast);
}

void ThunderSMTalonImpl::SetInverted(bool invert)
{
    talon.SetInverted(invert ? ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput : ctre::phoenix::motorcontrol::InvertType::None);
}






/**************************************************************/
// ThunderSMTalonImpl

ThunderSMCANImpl::ThunderSMCANImpl(int canid) : ThunderSparkMaxImpl("CAN SparkMax", canid),
        spark(canid, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
        enc(spark.GetEncoder()),
        smpid(spark.GetPIDController()),
        pid(&smpid)
{
}

void ThunderSMCANImpl::Set(double speed)
{
    spark.Set(speed);
    ThunderSparkMaxImpl::Set(speed);
}

void ThunderSMCANImpl::SetOpenLoopRampRate(double rate)
{
    spark.SetOpenLoopRampRate(rate);
}

double ThunderSMCANImpl::GetEncoder()
{
    return enc.GetPosition();
}

double ThunderSMCANImpl::Get() {
    return spark.GetAppliedOutput();
}

void ThunderSMCANImpl::SetEncoder(double rotations)
{
    enc.SetPosition(rotations);
}

void ThunderSMCANImpl::SetIdleMode(IdleMode idleMode)
{
    spark.SetIdleMode(idleMode == IdleMode::BRAKE ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
}

void ThunderSMCANImpl::SetInverted(bool invert)
{
    spark.SetInverted(invert);
}

void ThunderSMCANImpl::Follow(ThunderSparkMax *leader, bool invertOutput)
{
    ThunderSparkMaxImpl *impl = static_cast<ThunderSparkMaxImpl *>(leader);
    rev::CANSparkMax *lsm = impl->getSparkMax();
    if (lsm) {
        spark.Follow(*lsm, invertOutput);
    } else {
        printf("ERROR: CAN id %d cannot follow a fake spark max!\n", spark.GetDeviceId());
    }
}


/**************************************************************/
// ThunderSMPWMImpl

ThunderSMPWMImpl::ThunderSMPWMImpl(int pwmChannel) : ThunderSparkMaxImpl("PWM SparkMax", pwmChannel), spark(pwmChannel)
{
}

void ThunderSMPWMImpl::Set(double speed)
{
    spark.Set(speed);
    ThunderSparkMaxImpl::Set(speed);
}


/**************************************************************/
// ThunderSMFakeImpl

ThunderSMFakeImpl::ThunderSMFakeImpl() : ThunderSparkMaxImpl("PWM SparkMax", 0)
{
}

void ThunderSMFakeImpl::Set(double speed)
{
    ThunderSparkMaxImpl::Set(speed);
}

