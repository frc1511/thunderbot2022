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
public:
    Hang();
    ~Hang();

//functions
    //sendFeedback should be named debug. 
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    //hi ishan

    //pivots the extending arms forwards/backwards
    void pivot();
    //pivots the extending arms just a little bit so the arms hit the bar
    void reversePivot();
    //engages brake to stop arms from extending more
    void engageBrake();
    //hi jeff
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
    //pass hangState enum through function
    void changeState(HangState stage);
    //move functions to private
    //make stop command
    //make cimmand enumerator
    

private:
    //step that the robot is on in the overall process: mid, high, traversal, and what it is doing in general
    int step;
    //step it is in in the retract action
    int retractStep;
    //step it is in in the extend action
    int extendStep;
    //hang max height, dependent on how far it extended
    double hangMaxHeight;
    //height that hang should start slowing down
    double hangSlowDownHeight;
    //height hang should slow down even more
    double hangSlowDownMoreHeight;

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
    frc::DoubleSolenoid hangPivot{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_PIVOT_EXTEND, PCM1_HANG_PIVOT_RETRACT};
    frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, /*hi jeff*/PCM1_HANG_BRAKE_PISTON_EXTEND, PCM1_HANG_BRAKE_PISTON_RETRACT};
    //hi trevor

    //timer
    frc::Timer pivotTimer;
};