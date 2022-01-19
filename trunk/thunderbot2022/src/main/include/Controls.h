#pragma once

#include "Drive.h"

class Controls {
public:
    Controls(Drive* drive);
    ~Controls();

    void process();

private:
    Drive* drive;

    // Something here...
};