#pragma once

#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang);
    ~Controls();

    void process() override;

private:
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;

    // Something here...
};