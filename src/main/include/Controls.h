#pragma once

#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"

class Controls {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang, Limelight* limelight);
    ~Controls();

    void process();

private:
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;
    Limelight* limelight;

    // Something here...
};