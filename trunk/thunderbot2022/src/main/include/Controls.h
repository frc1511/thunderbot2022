#pragma once

#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"

class Controls {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang);
    ~Controls();

    void process();

private:
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;

    // Something here...
};