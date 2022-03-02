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
#include "BlinkyBlinky.h"
#include "Camera.h"
#include "RobotChess.h"
#include <frc/Compressor.h>
#include <frc/Timer.h>

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
  Controls controls { &drive, nullptr, nullptr };
  Autonomous autonomous { &drive, nullptr, &controls };

  static const int nMechanisms = 2;
  Mechanism *allMechanisms[nMechanisms] {
    /*&camera,*/ &limelight, &drive
  };
#else
  GamEpiece gamEpiece { &limelight };
  Hang hang {};


  BlinkyBlinky blinkyBlinky {};
  Controls controls { &drive, &gamEpiece, &hang };

  Autonomous autonomous { &drive, &gamEpiece, &controls };

  static const int nMechanisms = 4;
  Mechanism *allMechanisms[nMechanisms] = {
    /*&camera,*/ &limelight, &hang, &gamEpiece, &drive
  };

  RobotChess robotChess {};

  frc::Compressor compressor { frc::PneumaticsModuleType::CTREPCM };
#endif
  frc::Timer feedbackTimer;
};
