#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Feedback.h"
#include "Limelight.h"
#include "Interpolation.h"
#include "ShotMath.h"
#include "ThunderSparkMax.h"
#include <frc/Servo.h>
#include <frc/AnalogPotentiometer.h>
#include <vector>
#include <fstream>

// Minimum / maximum hood servo positions.
#define HOOD_MIN_POS 0.5547 //0.433
#define HOOD_MAX_POS (HOOD_MIN_POS + .19) //0.7

// The maximum RPM of the shooter wheels.
#define SHOOTER_MAX_RPM 2700 // 5700

#define PETERS_INTERPOLATION

class Shooter : public Mechanism {
public:
    Shooter(Limelight* limelight);
    ~Shooter();

    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    /**
     * Sets whether the shooter should be spinning.
     */
    void setShooterSpinup(bool shouldShoot);

    /**
     * Returns whether the hood position is at the desired position AND shooter is up to speed.
     */
    bool isShooterReady();

    /**
     * Sets the speed of the hood manually (from controls).
     */
    void setHoodManual(double speed);

    /**
     * Spins the shooter wheels in reverse for outtakeing
     */
    void spinInReverse(bool reverseOrNot);

    enum ShooterMode {
        ODOMETRY,          // Use camera to decide shot speed and angle
        FAR_LAUNCH_PAD,        // shooting with back of robot contacting the far launch pad
        NEAR_LAUNCH_PAD,   // shooting with back of robot contacting the near launch pad
        TARMAC_LINE,       // shooting with front edge of robot on line at outer edge of tarmac
        TARMAC,
        HIGH_HUB_SHOT,     // shooting with the front edge of the robot at the hub 
        LOW_HUB_SHOT,      // shooting with the front edge of the robot at the hub 
        MANUAL,            // will be controlled manually using manual function for the hood and
                           // the shooter is fixed for shooting from tarmac
    };

    /**
     * Set the mode of the shooter.
     */
    void setShooterMode(ShooterMode mode);

    // changes the manual speed by 100, great for testing, true will increase, false will decrease.
    void changeManualSpeed(bool increaseOrDecrease);

    enum OdometryMode {
        CRAZY_MATH,
        INTERPOLATION,
    };

    void setOdometryMode(OdometryMode mode);

    // :D
    void recordShooterValues();


private:
    Limelight* limelight;

    // Whether the shooter wheels should spin up to speed.
    bool wantToShoot = false;

    // The manual speed of the hood servo.
    double hoodSpeedManual = 0;

    double hoodSpeedManualLast = 0;

    // The target RPM of the shooting motors.
    double targetRPM = 0;

    // The target position of the hood.
    double targetHoodPosition = 0;

    OdometryMode odometryMode = INTERPOLATION;
    
#ifndef PETERS_INTERPOLATION
    
    double interpolation(double firstX, double firstY, double lastX,  double lastY, double distance);
    int goodNumber;
    double distance;
    std::vector<double> varsDistance = {};
    std::vector<double> varsHood = {};
    std::vector<double> xVarsSpeed = {};
    std::vector<double> varsSpeed = {};

#endif
    

    // 
    double manualRPM = 0;
    // true will reverse, false will not
    bool reverse = false;

    ShooterMode shooterMode = TARMAC_LINE;

    // right and left shooter motors
    ThunderSparkMax *shooterLeftMotor;
    ThunderSparkMax *shooterRightMotor;
    
    // right and left shooter PID controllers
    ThunderSparkMaxCANPIDController *shooterLeftPID;
    ThunderSparkMaxCANPIDController *shooterRightPID;

    // reads the potentiometer value, returns 0-1
    double readPotentiometer();

    // Do configuration of motor controllers to "power on" state
    void configureMotors();
    
    frc::Servo hoodServo {PWM_SHOOTER_HOOD_SERVO};
    frc::AnalogPotentiometer hoodPotentiometer {ANALOG_SHOOTER_HOOD_POTENTIOMETER};

#ifdef PETERS_INTERPOLATION

    ShotMath shotMath {};

    Interpolation<double, double> hoodInterpolation;
    Interpolation<double, double> rpmInterpolation;

#endif

//hi jeff :D
// trevor's weird comments below

/** function for it its reaqdy to shoot (done)
 * funtion to spin wheels
 * change what the servo is trying ot got to
 * way to manually change speed of shooter
 * way to manually change potentiometer position
 * hi jeff :D
 */
    // Something here...
    /**very very very similar shooter to 2020/2021, if your stuck on code look there
     * and/or ask trevor/ishan/peter
     * 
     * Has two motors, both sides of the flywheels. 
     * 
     * 
     * motors need to be set to a speed, either game piece gives them a speed or 
     * a map poisition or enum, yadayada doesnt matter. able to check and return the
     *  current speed(if gamEpiece is giving the speed originally)/if it's at 
     * speed(if gamEpiece is not giving speed just position/enum). 
     * TLDR: spins 2 flywheels, lets gamEpiece know if ready to shoot
     * 
     * Also has a potentiometer and servo duo.
     * 
     * servo is not normal setPosition, youve gotta set the servo to a positive
     *  and negative speed until its within the correct potentiometer range. gamEpiece
     * will use the same thing when shooting either use the recieved speed to figure out
     *  which preset position to use or have a second perameter. If its a map position/enum 
     * kinda deal then it tells it same way it does speed
     * 
     * THERETICAL(idk how its spelled): if we get enough limelight/graphs/math then 
     * we might want to be able to set a specific speed and hood position via two perameters
     * like doShootStuff(double spinSpeed, double hoodPosition); not 100% sure if we will 
     * need that but if you use that from the get go and let gamEpiece tell you speed and hood
     * then it would be less work for you. ALSO HELPFUL FEATURE FOR MANUAL CONTROL
     * 
     * 
     * please read entire thing and not skip to the bottom :D
     */


    //this is for storage but is in shooter now
      // two motors?
    /** 
     * one of the motors controls stage two and the transition wheel
     * 
     * want to look at the stage two sensor to see if there is a ball 
     * and stop the stage two wheels once it sees one
     * 
     * when game piece says so needs to move same stage two motor for 
     * the ball to go into the transition wheel. ngl i might combine intake 
     * and storage cause the first "storage" motor might be belted to the intake
     * and it would just make more sense for ball control to be all together
     */
};