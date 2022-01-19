#pragma once

#include "Intake.h"
#include "Storage.h"
#include "Shooter.h"

class GamEpiece {
public:
    GamEpiece();
    ~GamEpiece();

    void process();

private:
    Intake intake {};
    Storage storage {};
    Shooter shooter {};

    // Something here...
};