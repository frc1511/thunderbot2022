#include "Mechanism.h"
#include <frc/DriverStation.h>

void Mechanism::doPersistentConfiguration() {}

void Mechanism::resetToMode(MatchMode mode) {}

void Mechanism::sendFeedback() {}

void Mechanism::process() {}

Mechanism::MatchMode Mechanism::getCurrentMode() {
    if (frc::DriverStation::IsDisabled()) {
        return MODE_DISABLED;
    }
    else if (frc::DriverStation::IsAutonomous()) {
        return MODE_AUTO;
    } 
    else if (frc::DriverStation::IsTeleop()) {
        return MODE_TELEOP;
    }
    else if (frc::DriverStation::IsTest()) {
        return MODE_TEST;
    }
    else {
        return MODE_DISABLED;
    }
}