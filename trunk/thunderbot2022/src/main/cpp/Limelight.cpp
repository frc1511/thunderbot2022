#include "Limelight.h"

#define LIMELIGHT_ANGLE 29.3_deg

#define LIMELIGHT_HEIGHT 43_in

#define HUB_HEIGHT (8_ft + 8_in)

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
    Feedback::sendDouble("limelight", "distance (meters)", getDistance().value());
    Feedback::sendDouble("limelight", "vertical angle (radians)", getAngleVertical().value());
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

units::meter_t Limelight::getDistance(){
    if (!hasTarget()) {
        return -1_m;
    }
    return units::meter_t(units::meter_t(HUB_HEIGHT - LIMELIGHT_HEIGHT).value() / units::math::tan(LIMELIGHT_ANGLE + getAngleVertical()).value());
}