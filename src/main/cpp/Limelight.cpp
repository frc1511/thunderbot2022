#include "Limelight.h"

#define LIMELIGHT_ANGLE 90_rad

Limelight::Limelight() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
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