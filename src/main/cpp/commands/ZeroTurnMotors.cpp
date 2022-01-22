// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroTurnMotors.h"
#include "RobotContainer.h"

ZeroTurnMotors::ZeroTurnMotors() {}

// Called when the command is initially scheduled.
void ZeroTurnMotors::Initialize() {
  RobotContainer::drivetrain->SetAllPosition(0);
}

// Called repeatedly when this Command is scheduled to run
void ZeroTurnMotors::Execute() {}

// Called once the command ends or is interrupted.
void ZeroTurnMotors::End(bool interrupted) {}

// Returns true when the command should end.
bool ZeroTurnMotors::IsFinished() {
  return false;
}
