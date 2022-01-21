// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "Controls.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include "Feedback.h"

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  
private:
  Drive drive {};
  GamEpiece gamEpiece {};
  Hang hang {};
  Limelight limelight {};
  Controls controls { &drive, &gamEpiece, &hang, &limelight };
  Feedback feedback {};
};