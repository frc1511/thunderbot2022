// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  blinkyBlinky.process();
}

void Robot::AutonomousInit() {
  resetMechanisms(Mechanism::MODE_AUTO);
}

void Robot::AutonomousPeriodic() {
  camera.process();
  drive.process();
  autonomous.process();
}

void Robot::TeleopInit() {
  resetMechanisms(Mechanism::MODE_TELEOP);
}

void Robot::TeleopPeriodic() {
  controls.process();
  drive.process();
  //gamEpiece.process();
  hang.process();
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
  blinkyBlinky.process();
}

void Robot::resetMechanisms(Mechanism::MatchMode mode) {
  limelight.resetToMode(mode);
  drive.resetToMode(mode);
  //gamEpiece.resetToMode(mode);
  hang.resetToMode(mode);
  controls.resetToMode(mode);
  autonomous.resetToMode(mode);
  blinkyBlinky.resetToMode(mode);
  camera.resetToMode(mode);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
