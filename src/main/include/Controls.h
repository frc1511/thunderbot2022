#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include <frc/Joystick.h>

class Controls : public Mechanism {
public:
#ifdef HOMER
    Controls(Drive* drive);
#else
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang);
#endif

    ~Controls();

    void process() override;

private:
    Drive* drive;
#ifndef HOMER
    GamEpiece* gamEpiece;
    Hang* hang;
#endif

    bool wasDriveModeToggled = false;
    bool isFieldCentric = false;
    bool wasSlowModeToggled = false;
    bool slowModeEnabled = false;

    frc::Joystick controllerDriver{0};
    frc::Joystick controllerAux{1};
    // Something here...
};