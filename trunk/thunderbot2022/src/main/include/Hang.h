#pragma once
#include "IOMap.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include "rev/CANSparkMax.h"
#pragma GCC diagnostic pop
#include <frc/DoubleSolenoid.h>


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
    //does the important stuff
    void process();
    //sends functions to dashboard
    void debug();
    //resets variables
    void reset();
    //pivots hang
    void pivot();
    //pivots the arms backwards
    void reversePivot();
    //engages brake to stop arms
    void engageBrake();
    //disengages brake to release arms
    void disengageBrake();
    //retracts arms
    void retract();

    //bar that the robot is going to
    enum HangState{TRAVERSAL, HIGH, MID, NOT_ON_BAR};
    //enumerator variable thing
    HangState hangState;

private:
    //step that the robot is on in the motion to traversal;
    int step = 0;
    //whether the robot is on the bar
    bool isOnBar = false;

    //actuator stuff
    rev::CANSparkMax winchMotor{CAN_HANG_WINCH_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    frc::DoubleSolenoid hangPivot{frc::PneumaticsModuleType::CTREPCM, INTAKE_PIVOT_EXTEND, INTAKE_PIVOT_RETRACT};
    frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_LEFT_BRAKE_PISTON_EXTEND, PCM1_HANG_LEFT_BRAKE_PISTON_RETRACT};

};