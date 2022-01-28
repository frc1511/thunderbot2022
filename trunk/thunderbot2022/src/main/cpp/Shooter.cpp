#include "Shooter.h"
#include "rev/RelativeEncoder.h"

Shooter::Shooter() {

}

Shooter::~Shooter() {

}

void Shooter::resetToMode(MatchMode mode) {

}

void Shooter::sendFeedback() {

}

void Shooter::process() {

}

bool Shooter::readySetGo()  {
    return (shooterMotorLeft.GetEncoder().GetVelocity()  >  speedGoTo) && (shooterMotorRight.GetEncoder().GetVelocity()  >  speedGoTo);
}                   

void Shooter::setShooterSpeed(double speed) {

}

void Shooter::setHoodPosition(double position)  {

}

void Shooter::setHoodSpeed(double speed) {
    
}