#pragma once

#include "Mechanism.h"
#include "Feedback.h"

class Shooter : public Mechanism {
public:
    Shooter();
    ~Shooter();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
    // Something here...
};