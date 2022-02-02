#if 0
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

        virtual ThunderSparkMaxCANPIDController* getSparkPID();
        virtual double GetOutputCurrent();

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
        printedVelocityWarning(false)
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

ThunderSparkMaxCANPIDController* ThunderSparkMaxImpl::getSparkPID() {
    printWarning(&printedPIDWarning, "Native PID");
    return &fakePIDController;
}

double ThunderSparkMaxImpl::GetOutputCurrent()
{
    printWarning(&printedCurrentWarning, "Current");
    return 0;
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
    ThunderSparkMaxCANPIDControllerImpl(rev::CANPIDController *pid);

    virtual rev::CANError SetOutputRange(double min, double max, int slotID = 0);
    virtual rev::CANError SetFF(double gain, int slotID = 0);
    virtual rev::CANError SetP(double gain, int slotID = 0);
    virtual rev::CANError SetI(double gain, int slotID = 0);
    virtual rev::CANError SetD(double gain, int slotID = 0);
    virtual rev::CANError SetIZone(double IZone, int slotID = 0);
    virtual rev::CANError SetReference(double value, rev::ControlType ctrl, int pidSlot = 0,
            double arbFeedforward = 0, 
            rev::CANPIDController::ArbFFUnits arbFFUnits = rev::CANPIDController::ArbFFUnits::kVoltage);
private:
    rev::CANPIDController *pid;
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
        virtual ThunderSparkMaxCANPIDController *getSparkPID() { return &pid; }
        virtual double GetVelocity() { return enc.GetVelocity(); }

        virtual double GetOutputCurrent() { return spark.GetOutputCurrent();}

    private:
        rev::CANSparkMax spark;
        rev::CANEncoder enc;
        rev::CANPIDController smpid;
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


struct MotorConfig {
    enum ControllerType {
        Jaguar,
        Talon,
        PWMSparkMax,
        CANSparkMax
    } controllerType;

    int channelOrId;

    MotorConfig(ControllerType controllerType, int channelOrId) : controllerType(controllerType), channelOrId(channelOrId) {}
};


ThunderSparkMaxCANPIDController::ThunderSparkMaxCANPIDController()
{
}

rev::CANError ThunderSparkMaxCANPIDController::SetOutputRange(double min, double max, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetFF(double gain, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetP(double gain, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetI(double gain, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetD(double gain, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetIZone(double IZone, int slotID)
{
    return rev::CANError::kOk;
}

rev::CANError ThunderSparkMaxCANPIDController::SetReference(double value, rev::ControlType ctrl, int pidSlot,
        double arbFeedforward, 
        rev::CANPIDController::ArbFFUnits arbFFUnits)
{
    return rev::CANError::kOk;
}

ThunderSparkMaxCANPIDControllerImpl::ThunderSparkMaxCANPIDControllerImpl(rev::CANPIDController *pid) : pid(pid)
{
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetOutputRange(double min, double max, int slotID)
{
    return pid->SetOutputRange(min, max, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetFF(double gain, int slotID)
{
    return pid->SetFF(gain, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetP(double gain, int slotID)
{
    return pid->SetP(gain, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetI(double gain, int slotID)
{
    return pid->SetI(gain, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetD(double gain, int slotID)
{
    return pid->SetD(gain, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetIZone(double IZone, int slotID)
{
    return pid->SetIZone(IZone, slotID);
}

rev::CANError ThunderSparkMaxCANPIDControllerImpl::SetReference(double value, rev::ControlType ctrl, int pidSlot,
        double arbFeedforward, 
        rev::CANPIDController::ArbFFUnits arbFFUnits)
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
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_LF),    // DriveFrontLeft
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_RF),    // DriveFrontRight,
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_LR),    // DriveRearLeft,
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_RR),    // DriveRearRight,

        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_INTAKE_PIVOT),   // IntakePivot,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_INTAKE_BEATERBARS),   // IntakeBeaterBars,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_HELIX),    // StorageAgitator,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_STORAGE_SHOOT_TRANSITION),   // StorageTransitionToShooter,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SHOOT_PRIMER),   // ShooterPrimer,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SHOOT_LEFT),   // ShooterLeft,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_SHOOT_RIGHT),   // ShooterRight,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_CONTROL_SPINNER),   // WheelOfFortune,
        MotorConfig(MotorConfig::ControllerType::CANSparkMax, CAN_HANG_WINCH),   // Hang,
#else
        // Next 2 are fixed as CAN talon for At home testing with encoders
        MotorConfig(MotorConfig::ControllerType::Talon, 3),    // DriveFrontLeft
        MotorConfig(MotorConfig::ControllerType::Talon, 5),    // DriveFrontRight,
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_LR),    // DriveRearLeft,
        MotorConfig(MotorConfig::ControllerType::PWMSparkMax, PWM_DRIVE_RR),    // DriveRearRight,

        // As labelled on board
        //MotorConfig(MotorConfig::ControllerType::CANSparkMax, 10),   // IntakePivot,
        // at home testing
        MotorConfig(MotorConfig::ControllerType::Jaguar, 10),   // IntakePivot,
        MotorConfig(MotorConfig::ControllerType::Talon, 8),   // IntakeBeaterBars,
        MotorConfig(MotorConfig::ControllerType::Jaguar, 2),    // StorageAgitator,
        MotorConfig(MotorConfig::ControllerType::Talon, 2),   // StorageTransitionToShooter,
        MotorConfig(MotorConfig::ControllerType::Jaguar, 3),   // ShooterPrimer,
        // As labelled
        //MotorConfig(MotorConfig::ControllerType::CANSparkMax, 9),   // ShooterLeft,
        // at home testing
        MotorConfig(MotorConfig::ControllerType::Jaguar, 11),   // ShooterLeft,
        // As labelled
        //MotorConfig(MotorConfig::ControllerType::Talon, 3),   // ShooterRight,
        // At home testing
        MotorConfig(MotorConfig::ControllerType::Jaguar, 6),   // ShooterRight,
        MotorConfig(MotorConfig::ControllerType::Talon, 6),   // WheelOfFortune,
        // As labelled
        //MotorConfig(MotorConfig::ControllerType::Talon, 5),   // Hang,
        // At home testing
        MotorConfig(MotorConfig::ControllerType::Jaguar, 8),   // Hang,
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
// ThunderSMJagImpl

ThunderSMPWMImpl::ThunderSMPWMImpl(int pwmChannel) : ThunderSparkMaxImpl("PWM SparkMax", pwmChannel), spark(pwmChannel)
{
}

void ThunderSMPWMImpl::Set(double speed)
{
    spark.Set(speed);
    ThunderSparkMaxImpl::Set(speed);
}

#endif