#pragma once
#include "Mechanism.h"
#include "IOMap.h"
#include "rev/CANSparkMax.h"
#include <frc/DoubleSolenoid.h>
//hi jeff
#include <frc/Encoder.h>
#include <frc/DigitalSource.h>
#include <frc/DigitalInput.h>
#include "Feedback.h"
#include <frc/Timer.h>
#include <frc/Servo.h>

/*
PART 1
    -ACTUATORS
        -NEO
        -pivot piston
        -brake piston

    -SENSORS
        -optical sensor at bottom inside hang to check if fully retracted
        -flag sensor(2x - one on each arm) to check if retracted
        -encoder on winch to get exact axle measurement

    -DRIVER FEEDBACK
        -which bar robot is on/robot moving between bars
            -on dashboard
        -whether the robot is moving between bars or on the bar safely

PART 2
    COMMANDS:
        MID BAR:
            -disengage brake-brake piston
            -once fully extended, retract until fully retracted - optical sensor to know when retracted,
                flag sensors to know if extended
            -engage brake - brake piston
        HIGH/TRAVERSAL BAR:
            -disengage brake-brake piston
            -slightly extend arm-encoder
            -engage brake-brake piston
            -pivot arm all the way-pivot piston
            -disengage brake-brake piston
            -pivot a tiny bit to hit high bar-dont know yet-probably slow exhaust?
            -retract until fully retracted- know if extended with flag sensor, 
                know if retracted with optical sensor
            -engage brake-brake piston
*/

class Hang : public Mechanism {

private:
    //manual enumerator for actions
    enum Manual{EXTEND, RETRACT, EXTEND_A_LITTLE, PIVOT, REVERSE_PIVOT, ENGAGE_BRAKE, DISENGAGE_BRAKE};
    //enumerator variable thing
    Manual manual;
    //bar that the robot is going to
    enum HangState{TRAVERSAL, HIGH, MID, NOT_ON_BAR, STOP};
    //enumerator variable thing
    HangState targetStage;
    //pivots the extending arms forwards/backwards
    void pivot();
    //engages the hard stop to keep the extending arms from flopping like trevor when he refuses to stand up
    void engageHardStop();
    //disengages the hard stops
    void disengageHardStop();
    //retract function if sensors broke
    void brokenRetract();
    //extend function if sensors broke
    void brokenExtend();
    //extendALittle function if sensors broke
    void brokenExtendALittle();
    //pivots the extending arms just a little bit so the arms hit the bar
    void reversePivot();
    //engages brake to stop arms from extending more
    void engageBrake();
    //hi jeff
    //disengages brake to extend arms
    void disengageBrake();
    //retracts arms
    void retract();
    //extends arms fully
    void extend();
    //extends the extending arms a little to get off the bar
    void extendALittle();
    //step that the robot is on in the overall process: mid, high, traversal, and what it is doing in general
    int step;
    //broken step 
    int brokenStep;
    //step it is in in the retract action
    int retractStep;
    //step it is in in the extend action
    int extendStep;
    //step for disengaging the brake
    int disengageBrakeStep;
    //step for the extendALittle function
    double hangMaxHeight;
    //height that hang should start slowing down
    double hangSlowDownHeight;
    //height hang should slow down even more
    double hangSlowDownMoreHeight;

//sensors that are not servos
    //beam break to tell if the pawl is disengaged
    frc::DigitalInput rachetBeamBreak {DIO_HANG_RATCHET_BEAM_BREAK};
    //encoder on winch to tell how far its gone
    frc::Encoder winchEncoder {CAN_HANG_ENCODER_A,  CAN_HANG_ENCODER_B};
    //flag sensor on the left arm to tell when its extended	
    frc::DigitalInput leftFlag {DIO_HANG_FLAG_SENSOR_LEFT};
    //flag sensor on the right arm to tell when its extended
    frc::DigitalInput rightFlag {DIO_HANG_FLAG_SENSOR_RIGHT};
    //sensor on the bottom of the arm to tell when fully retracted, doesnt matter which
    //could also be on the winch i dont know yet
    frc::DigitalInput homeSensor {DIO_HANG_OPTICAL_HOME_SENSOR};

    //actuator stuff
    rev::CANSparkMax winchMotor{CAN_HANG_WINCH_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    frc::DoubleSolenoid hangPivot{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_PIVOT_1_EXTEND, PCM1_HANG_PIVOT_1_RETRACT};
    frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, /*hi jeff*/PCM1_HANG_BRAKE_PISTON_EXTEND, PCM1_HANG_BRAKE_PISTON_RETRACT};
    //servos
    frc::Servo leftServo{PWM_HANG_LEFT_SERVO_STOP};
    frc::Servo rightServo{PWM_HANG_RIGHT_SERVO_STOP};
    frc::Servo ratchetServo{PWM_HANG_RACHET_AND_PAWL};//DONT KNOW WHY BUT THE RATCHET AND PAWL FROM IOMAP DOESNT WORK

    //hi trevor

    //timer
    frc::Timer hangTimer;

    public:
    Hang();
    ~Hang();

//functions
    //sendFeedback should be named debug. 
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    //hi ishan

    //command to change the target bar
    void targetBar(HangState stage);
    void commandManual(Manual manualCommands);
    //make command enum
};