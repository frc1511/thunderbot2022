#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Feedback.h"
#include "Limelight.h"
#include "ThunderSparkMax.h"
#include <frc/Servo.h>
#include <frc/AnalogPotentiometer.h>

class Shooter : public Mechanism {
public:
    Shooter(Limelight* limelight);
    ~Shooter();

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

    enum ShooterMode {
        ODOMETRY,      // Use camera to decide shot speed and angle
        LAUNCH_PAD,    // shooting with back of robot contacting launch pad
        TARMAC_LINE,   // shooting with front edge of robot on line at outer edge of tarmac 
        MANUAL,        // will be controlled manually using manual function for the hood and
                       // the shooter is fixed for shooting from tarmac
    };

    /**
     * Set the mode of the shooter.
     */
    void setShooterMode(ShooterMode mode);

private:
    Limelight* limelight;

    // Whether the shooter wheels should spin up to speed.
    bool wantToShoot = false;

    // The manual speed of the hood servo.
    double hoodSpeedManual = 0;

    // The target RPM of the shooting motors.
    double targetRPM = 0;

    // The target position of the hood.
    double targetHoodPosition = 0;

    ShooterMode shooterMode = TARMAC_LINE;

    // right and left shooter motors
    ThunderSparkMax *shooterLeftMotor;
    ThunderSparkMax *shooterRightMotor;

    // right and left shooter PID controllers
    ThunderSparkMaxCANPIDController *shooterLeftPID;
    ThunderSparkMaxCANPIDController *shooterRightPID;
    
    frc::Servo hoodServo {PWM_SHOOTER_HOOD_SERVO};
    frc::AnalogPotentiometer hoodPotentiometer {ANALOG_SHOOTER_HOOD_POTENTIOMETER};

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