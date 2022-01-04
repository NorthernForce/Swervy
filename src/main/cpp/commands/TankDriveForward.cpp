// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDriveForward.h"
#include "RobotContainer.h"

TankDriveForward::TankDriveForward() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TankDriveForward::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDriveForward::Execute() {
    RobotContainer::drivetrain->TankDrive(0.3, 0.3);
}

// Called once the command ends or is interrupted.
void TankDriveForward::End(bool interrupted) {
    RobotContainer::drivetrain->SetAllDriveSpeed(0);
}

// Returns true when the command should end.
bool TankDriveForward::IsFinished() {
  return false;
}
