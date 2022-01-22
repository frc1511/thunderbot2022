#pragma once

#include "Mechanism.h"
#include "Feedback.h"

class Intake : public Mechanism {
public:
    Intake();
    ~Intake();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
        // Something here...
};