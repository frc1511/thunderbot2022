#pragma once

#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive, GamEpiece* gamEpiece);
    ~Autonomous();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
    Drive* drive;
    GamEpiece* gamEpiece;
};