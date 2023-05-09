#pragma once

class ShotCalculus {
public:
    ShotCalculus();
    ~ShotCalculus();
    
    void Feedback();

    struct Shot {
        double hoodPos = 0;
        double shooterRPM = 0;
    };

    Shot calculateShot(double distance, double vel, double angle);

    
    void calculateAngleCompensation(double xVel); //needs other stuff
private:
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    double xDistance = 0;
    double xVelocity = 0;
    double yDistance = 0;
    double yVelocity = 0;
    double time = 0;
    double A = 0;
    double B = 0;
    double C = 0;
    double finalAngle = 0;
    double finalSpeed = 0;
    double finalHoodAngle = 0;
    double finalShotRPM = 0;
};