// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithJoystick.h"
#include "RobotContainer.h"

DriveWithJoystick::DriveWithJoystick() {
    AddRequirements(RobotContainer::drivetrain.get());
}

void DriveWithJoystick::Initialize() {}

void DriveWithJoystick::Execute() {
    auto driveControls = RobotContainer::oi->GetDriveControls();
    speed = driveControls[0];
    strafe = driveControls[1];
    yaw = driveControls[2];

    RobotContainer::drivetrain->SwerveDrive(speed, strafe, yaw);
}

void DriveWithJoystick::End(bool interrupted) {
    RobotContainer::drivetrain->StopAll();
}

bool DriveWithJoystick::IsFinished() {
    return false;
}
