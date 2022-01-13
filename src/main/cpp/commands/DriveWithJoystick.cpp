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
    yaw = driveControls[1];
    strafe = driveControls[2];
    RobotContainer::drivetrain->SetDrive(units::velocity::meters_per_second_t(-strafe), units::velocity::meters_per_second_t(speed), units::radians_per_second_t(yaw), false);
    //RobotContainer::drivetrain->SetDrive(speed, strafe, yaw);
}

void DriveWithJoystick::End(bool interrupted) {
    RobotContainer::drivetrain->StopAll();
}

bool DriveWithJoystick::IsFinished() {
    return false;
}
