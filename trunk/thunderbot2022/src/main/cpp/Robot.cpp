#include "Robot.h"

bool isCraterMode = false;

void Robot::RobotInit() {
    feedbackTimer.Reset();
    feedbackTimer.Start();
}
void Robot::RobotPeriodic() {
    if(feedbackTimer.Get().value() >= .5){
        for (int i = 0; i < nMechanisms; ++i)
            allMechanisms[i]->sendFeedback();
        autonomous.sendFeedback();
        controls.sendFeedback();
        feedbackTimer.Reset();
    }
}

void Robot::AutonomousInit() {
    reset(Mechanism::MODE_AUTO);
    //controls.autoForTrevor();
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
    controls.controllerInDisable();
}

void Robot::TestInit() {
    if (controls.getShouldPersistConfig()) {
        printf("*** Persistent configuration activating...\n");
        for (int i = 0; i < nMechanisms; ++i) {
            allMechanisms[i]->doPersistentConfiguration();
        }
        printf("*** Persistent configuration complete!\n");
    }
    reset(Mechanism::MODE_TEST);
}

void Robot::TestPeriodic() {
    // drive.manualDrive(0, .1, 0);
    // drive.process();
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
