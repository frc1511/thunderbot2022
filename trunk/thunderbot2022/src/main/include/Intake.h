#pragma once

#include "Mechanism.h"
#include "Feedback.h"
/** has two sensors and two motors
 * stage one banner sensor and stage two banner sensor
 * and two double solonoids
 */
class Intake : public Mechanism {
public:
    Intake();
    ~Intake();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    enum IntakeDirection{
        INTAKE,
        OUTTAKE,
        NOTTAKE,
        FORCE_STAGE_TWO
    };
    void setIntakeDirection(IntakeDirection intakeDirection);
    void setIntakePosition(bool position); // true is down, false is up
    void setIntakeSpeed(double speed); // positive to intake, negative to outtake, remember to make sure thing is down before you spin it :D
private:
    
        // Something here...
};