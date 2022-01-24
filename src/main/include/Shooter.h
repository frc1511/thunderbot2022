#pragma once

#include "Mechanism.h"
#include "Feedback.h"

class Shooter : public Mechanism {
public:
    Shooter();
    ~Shooter();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
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