// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "IOMap.h"
#include "Controls.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include "Feedback.h"
#include "Autonomous.h"
//#include "AutoForTrevor.h"
//#include "ControlsForTrevor.h"
#include "BlinkyBlinky.h"
#include "Camera.h"

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

  void reset(Mechanism::MatchMode mode);
  void sendFeedback();
  
private:
  Camera camera {};
  Limelight limelight {};
  Drive drive { &camera, &limelight };

#ifdef HOMER
  Controls controls { &drive };
#else
  GamEpiece gamEpiece { &limelight };
  Hang hang {};
  
  //ControlsForTrevor controlsForTrevor{};
  //AutoForTrevor autoForTrevor{};
  Autonomous autonomous { &drive, &gamEpiece };

  BlinkyBlinky blinkyBlinky {};
  Controls controls { &drive, &gamEpiece, &hang };
#endif

};
