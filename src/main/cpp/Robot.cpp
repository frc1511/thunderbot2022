#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
    for (int i = 0; i < nMechanisms; ++i)
        allMechanisms[i]->sendFeedback();
    autonomous.sendFeedback();
    controls.sendFeedback();
}

void Robot::AutonomousInit() {
    reset(Mechanism::MODE_AUTO);
}

void Robot::AutonomousPeriodic() {
    autonomous.process();
    for (int i = 0; i < nMechanisms; ++i)
        allMechanisms[i]->process();
}

void Robot::TeleopInit() {
    reset(Mechanism::MODE_TELEOP);
}

void Robot::TeleopPeriodic() {
    controls.process();
    for (int i = 0; i < nMechanisms; ++i)
        allMechanisms[i]->process();
}

void Robot::DisabledInit() {
    reset(Mechanism::MODE_DISABLED);
}

void Robot::DisabledPeriodic() {
}

void Robot::TestInit() {
    reset(Mechanism::MODE_TEST);
}

void Robot::TestPeriodic() {
}

void Robot::reset(Mechanism::MatchMode mode) {
    for (int i = 0; i < nMechanisms; ++i)
        allMechanisms[i]->resetToMode(mode);
    autonomous.resetToMode(mode);
    controls.resetToMode(mode);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
