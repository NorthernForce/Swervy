// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/Command.h>
#include <subsystems/Drivetrain.h>
#include <OI.h>
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();
  void InitAutonomousCommands();
  static std::shared_ptr<OI> oi;
  static std::shared_ptr<Drivetrain> drivetrain;

 private:
  void InitSubsystems();
  void InitDefaultCommands();
};
