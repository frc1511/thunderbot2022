// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  sendFeedback();
#ifndef HOMER
  blinkyBlinky.process();
#endif
}

void Robot::AutonomousInit() {
  reset(Mechanism::MODE_AUTO);
}

void Robot::AutonomousPeriodic() {
  limelight.process();
  drive.process();
  camera.process();
#ifndef HOMER
  autonomous.process();
#endif
}

void Robot::TeleopInit() {
  reset(Mechanism::MODE_TELEOP);
}

void Robot::TeleopPeriodic() {
  controls.process();
  limelight.process();
  drive.process();
#ifndef HOMER
  //gamEpiece.process();
  hang.process();
#endif
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
  // Why would we use test?
  
#ifndef HOMER
  blinkyBlinky.process();
#endif
}

void Robot::reset(Mechanism::MatchMode mode) {
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

void Robot::sendFeedback() {
  limelight.sendFeedback();
  drive.sendFeedback();
  camera.sendFeedback();
#ifndef HOMER
  //gamEpiece.sendFeedback();
  hang.sendFeedback();
  controls.sendFeedback();
  autonomous.sendFeedback();
  blinkyBlinky.sendFeedback();
#endif
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
