
#pragma once

#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>

/*
 * Base class for all on-robot mechanisms
 */
class Mechanism {

public:
    typedef enum {
        MODE_DISABLED,
        MODE_AUTO,
        MODE_TELEOP,
        MODE_TEST
    } MatchMode;

    /**
     * Reset this mechanism in preparation to run in the given mode of a match
     * This should reset all internal state to make the system ready to operate
     * in the given match mode on the next set of commands.  Generally speaking,
     * a mechanism will abandon all commands in progress when this is called
     * and reset to a default state.
     */
    virtual void resetToMode(MatchMode mode);

    /**
     * Send operational and/or diagnostic feedback to the operator of the robot.
     * This is called periodically in all modes of competition.
     */
    virtual void sendFeedback();

    /**
     * Mechanisms should implement this to perform periodic actions in response to
     * commands given to the mechanism. This is called periodically in all modes
     * of competition except for disabled.
     */
    virtual void process() = 0;

protected:
    MatchMode getCurrentMode();

private:
    MatchMode currentMode{MODE_DISABLED};
};

