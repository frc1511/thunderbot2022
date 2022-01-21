#pragma once
#include "IOMap.h"
#include "rev/CANSparkMax.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <frc/DigitalSource.h>
#include <frc/DigitalInput.h>

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

class Hang {
public:
    Hang();
    ~Hang();

//functions
    //does the important stuff to make the mechanism work
    void process();
    //sends stuff to dashboard for debugging purposes
    void debug();
    //resets variables
    void reset();
    //pivots the extending arms forwards/backwards
    void pivot();
    //pivots the extending arms just a little bit so the arms hit the bar
    void reversePivot();
    //engages brake to stop arms from extending more
    void engageBrake();
    //disengages brake to extend arms
    void disengageBrake();
    //retracts arms
    void retract();
    //extends arms
    void extend();

    //bar that the robot is going to
    enum HangState{TRAVERSAL, HIGH, MID, NOT_ON_BAR};
    //enumerator variable thing
    HangState hangState;

    enum IsHangWorking{BROKEN, FUNCTIONAL};
    IsHangWorking isHangWorking;

private:
    //step that the robot is on in the motion to traversal
    int step = 0;
    //step to reach mid bar
    int midBarStep = 0;
    //whether the robot is on the bar
    bool isOnBar = false;

//sensors
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
    frc::DoubleSolenoid hangPivot{frc::PneumaticsModuleType::CTREPCM, INTAKE_PIVOT_EXTEND, INTAKE_PIVOT_RETRACT};
    frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_LEFT_BRAKE_PISTON_EXTEND, PCM1_HANG_LEFT_BRAKE_PISTON_RETRACT};

};