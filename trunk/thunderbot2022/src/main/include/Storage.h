#pragma once

#include "Mechanism.h"
#include "Feedback.h"

class Storage : public Mechanism {
public:
    Storage();
    ~Storage();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

private:
    // Something here...
};