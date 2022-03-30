#pragma once
// RIP "#pragma twice", Forever in our hearts, Revision 68 to Revision 107 - trevor wiesen

#include "Mechanism.h"
#include "IOMap.h"
#include "ThunderSparkMax.h"
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
//sensors that are not servos
    //encoder on winch to tell how far its gone
#ifdef TEST_BOARD
    frc::Encoder winchEncoder {DIO_HANG_ENCODER_A,  DIO_HANG_ENCODER_B};//change to something else
#endif
    /*sensor on the bottom of the arm to tell when fully retracted, doesnt matter which
    could also be on the winch i dont know yet*/
    frc::DigitalInput homeSensor {DIO_HANG_OPTICAL_HOME_SENSOR};
    //actuator stuff
    ThunderSparkMax *winchMotor;
    //TOP PISTON CONNECTING hangPivot2 to the arm
    frc::DoubleSolenoid hangPivot1{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_PIVOT_1_EXTEND_PISTON, PCM1_HANG_PIVOT_1_RETRACT_PISTON};
    //frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, /*hi jeff*/PCM1_HANG_BRAKE_PISTON_EXTEND, PCM1_HANG_BRAKE_PISTON_RETRACT};
    //connects the robot to hangPivot1
    frc::DoubleSolenoid hangPivot2{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_PIVOT_2_EXTEND_PISTON, PCM1_HANG_PIVOT_2_RETRACT_PISTON};
#ifndef TEST_BOARD
    frc::DoubleSolenoid staticPivot{frc::PneumaticsModuleType::CTREPCM, PCM1_HANG_STATIC_PISTON_EXTEND, PCM1_HANG_STATIC_PISTON_RETRACT};
#endif

    //servos
    frc::Servo ratchetServo{PWM_HANG_RACHET_AND_PAWL};
    frc::Servo stringServoRight{PWM_STRING_SERVO_RIGHT};
    frc::Servo stringServoLeft{PWM_STRING_SERVO_LEFT};
#ifndef TEST_BOARD
    frc::DigitalInput staticSensor {DIO_HANG_REFLECTIVE_SENSOR};
#endif

    //hi trevor

    //timer
    frc::Timer hangTimer;
    bool isDone;
    bool stepDone;
    int hangBar = 0;
    double manualStep;
    bool test;
    double highOrTraversal;
    double currentEncoderValue;
    //pivots the extending arms forwards/backwards
    void pivot(bool armsForward);//working
    //reads encoder
    double readEncoder();
    //low bar extend
    void lowExtend();
    //sets encoder
    void resetEncoder();
    //retract function if sensors broke
    void brokenRetract();
    //extend function if sensors broke
    void brokenExtend();
    //extendALittle function if sensors broke
    void brokenExtendALittle();
    //pivots the extending arms just a little bit so the arms hit the bar
    void reversePivot();//working
    //engages brake to stop arms from extending more
    void engageBrake();//working
    //winds up the string to pull the arms together
    void windUpString();
    //unwinds the string
    void unwindString();
    //hi jeff
    //disengages brake to extend arms
    bool disengageBrake();//working
    //retracts arms
    void retract();
    //extends arms fully
    void extend();//working
    //extends the extending arms a little to get off the bar
    void extendALittle();//working
    //configure motor
    void configureMotor();
    //modified retract function for high/traversal post-flr
    void retractForHigh();
    //retracts from max extension to halfway
    void retractMaxToHalf();
    //retracts from half extension to fully in
    void retractHalfToFull();
    //retracts from half extension to fully in and switches to moving arms
    void retractHalfToStaticArms();
    //step that the robot is on in the overall process: mid, high, traversal, and what it is doing in general
    int step;
    //broken step 
    int brokenStep;
    //step it is in in the retract action
    int retractStep;
    //step it is in in the extend action
    int extendStep;
    //step for the extendALittle function
    double hangMaxHeight;
    //broken retract doubles for height
    double hangBrokenMaxHeight;
    double hangBrokenSlowHeight;
    double hangBrokenSlowerHeight;
    //is the robot done/on traversal
    bool autoDone;
    bool extendALittleDone;
    double disengageBrakeStart;
    bool disengageBrakeDone;
    bool retractDone;
    bool retractCurrentIncrease;
    bool highDone;
    bool pauseEnabled;
    bool pauseDisabled;

    public:
   
    bool getIsLow();
    void setGoingForHigh(bool highOrNot);
    void setPause(bool paused, bool unpaused);
    bool goingForHigh; // true is going for high, false is going for mid
    bool isLowHeight;
    void setIsLow(bool isLow);

    //manual enumerator for actions
    enum Manual{EXTEND,  
                EXTEND_A_LITTLE, 
                PIVOT_IN,
                PIVOT_OUT,
                REVERSE_PIVOT, 
                ENGAGE_BRAKE, 
                DISENGAGE_BRAKE,
                PULL_STRING, 
                UNWIND_STRING,
                RETRACT,
                NOT,
                DRIVE_DOWN};

    //enumerator variable thing
    Manual manual;
    Manual currentManualState;
    //bar that the robot is going to
    enum HangState{HIGH_TRAVERSAL, MID, MID_2, NOT_ON_BAR, PAUSE, LOW, LOW_2};
    //enumerator variable thing
    HangState targetStage;
    HangState completedTargetStage;

    enum ExtendLevel{LOW_HEIGHT, MID_HEIGHT, HIGH_TRAVERSAL_HEIGHT};
    ExtendLevel extendLevel;


    Hang();
    ~Hang();

//functions
    //sendFeedback should be named debug. 
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

//mutually exclusive
    //command to change the target bar
    void commandAuto();
    // command to override the current action
    void setCommandAutoOverride();
    //command to do manual actions
    void commandManual(Manual manualCommands);
    //command to set height of moving arm extension
    void commandHeight(ExtendLevel extendLevelCommand);
};