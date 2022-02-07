// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  drive.sendFeedback();
#ifndef HOMER
  blinkyBlinky.process();
#endif
}

void Robot::AutonomousInit() {
  resetMechanisms(Mechanism::MODE_AUTO);
}

void Robot::AutonomousPeriodic() {
  drive.process();
#ifndef HOMER
  camera.process();
  autonomous.process();
#endif
}

void Robot::TeleopInit() {
  resetMechanisms(Mechanism::MODE_TELEOP);
}

void Robot::TeleopPeriodic() {
  controls.process();
  drive.process();
#ifndef HOMER
  //gamEpiece.process();
  hang.process();
#endif
}

void Robot::DisabledInit() {
  resetMechanisms(Mechanism::MODE_DISABLED);
}

void Robot::DisabledPeriodic() {

}

void Robot::TestInit() {
  resetMechanisms(Mechanism::MODE_TEST);
}

void Robot::TestPeriodic() {
  // Why would we use test?
#ifndef HOMER
  blinkyBlinky.process();
#endif
}

void Robot::resetMechanisms(Mechanism::MatchMode mode) {
  limelight.resetToMode(mode);
  drive.resetToMode(mode);
  camera.resetToMode(mode);
#ifndef HOMER
  //gamEpiece.resetToMode(mode);
  hang.resetToMode(mode);
  controls.resetToMode(mode);
  autonomous.resetToMode(mode);
  blinkyBlinky.resetToMode(mode);
#endif
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
