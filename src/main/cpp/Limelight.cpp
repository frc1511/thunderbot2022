#include "Limelight.h"

#define LIMELIGHT_ANGLE 30  // degrees

#define LIMELIGHT_HEIGHT 43 // inches

#define HUB_HEIGHT 12 // inches

Limelight::Limelight() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-homer");
}

Limelight::~Limelight() {

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