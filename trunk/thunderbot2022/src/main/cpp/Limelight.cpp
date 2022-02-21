#include "Limelight.h"

#define LIMELIGHT_ANGLE 30  // degrees

#define LIMELIGHT_HEIGHT 43 // inches

#define HUB_HEIGHT 12 // inches

Limelight::Limelight() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-homer");
}

Limelight::~Limelight() {

}

void Limelight::resetToMode(MatchMode mode) {
    switch (mode) {
        case MODE_DISABLED:
        case MODE_TEST:
            setLEDMode(LEDMode::OFF);
            break;
        case MODE_AUTO:
        case MODE_TELEOP:
            setLEDMode(LEDMode::ON);
            break;
        }
}

void Limelight::sendFeedback() {
    Feedback::sendDouble("limelight", "distance (in?)", getDistance());
    Feedback::sendDouble("limelight", "vertical angle", getAngleVertical().value());
}

bool Limelight::hasTarget() {
    return table->GetNumber("tv", 0.0);
}

units::radian_t Limelight::getAngleHorizontal() {
    return units::degree_t(table->GetNumber("tx", 0.0));
}

units::radian_t Limelight::getAngleVertical() {
    return units::degree_t(table->GetNumber("ty", 0.0));
}

void Limelight::setLEDMode(LEDMode mode) {
    table->PutNumber("ledMode", (int)mode);
}

Limelight::LEDMode Limelight::getLEDMode() {
    return (LEDMode)(int)table->GetNumber("ledMode", 0.0);
}

void Limelight::setCameraMode(CameraMode mode) {
    table->PutNumber("camMode", (int)mode);
}

Limelight::CameraMode Limelight::getCameraMode() {
    return (CameraMode)(int)table->GetNumber("camMode", 0.0);
}

double Limelight::getDistance(){
    return (HUB_HEIGHT - LIMELIGHT_HEIGHT) / tan(LIMELIGHT_ANGLE + units::degree_t(getAngleVertical()).value());
}